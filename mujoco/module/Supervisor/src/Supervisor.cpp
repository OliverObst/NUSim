#include "module/Supervisor/src/Supervisor.hpp"

#include <vector>

#include "module/Supervisor/src/GameControllerPacket.hpp"
#include "module/Supervisor/src/SupervisorConfig.hpp"
#include "shared/message/SimMessages.hpp"
#include "shared/util/Config.hpp"

namespace k1sim::module {

    namespace gc = k1sim::module::supervisor::gc;
    using k1sim::module::supervisor::SupervisorConfig;
    using k1sim::module::supervisor::SupervisorLogic;

    namespace {

        SupervisorConfig build_config() {
            return k1sim::module::supervisor::load_config(config::load("supervisor.yaml"));
        }

    }  // namespace

    Supervisor::Supervisor(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        // Loaded eagerly (constructor body, not Startup) so the port number is
        // known before the on<UDP> reaction below is installed — mirrors
        // module::Simulation's build_sim_config(), the established pattern in
        // this tree for config that has to be ready before reactions bind
        // (this port isn't hot-reloadable; NUClear's on<Configuration> extension
        // isn't used anywhere in this MuJoCo port, see shared/util/Config.hpp).
        SupervisorConfig cfg = build_config();
        const bool enabled   = cfg.enabled;
        const int gc_port    = cfg.gc_port;
        logic_               = std::make_unique<SupervisorLogic>(std::move(cfg));

        on<Trigger<message::SimHandles>>().then([this](const message::SimHandles& handles) {
            placed_robot_ = handles.placed_robot;
            reset_world_  = handles.reset_world;
            model_.store(handles.model, std::memory_order_release);
            data_.store(handles.data, std::memory_order_release);
            sim_mutex_.store(handles.mutex, std::memory_order_release);
        });

        if (!enabled) {
            on<Startup>().then(
                [this] { log<NUClear::LogLevel::INFO>("Supervisor disabled (supervisor.yaml: enabled: false)"); });
            on<Shutdown>().then([this] { log<NUClear::LogLevel::INFO>("Supervisor shutting down"); });
            return;
        }

        // Plain on<UDP> (unicast-style bind to any local address) rather than
        // on<UDP::Broadcast>: the latter only accepts datagrams whose
        // *destination* address is itself a broadcast address (255.255.255.255,
        // or an interface's configured broadcast address) — real GameController
        // traffic qualifies, but a directly-addressed test packet to
        // 127.0.0.1:gc_port (the loopback interface has no broadcast address at
        // all on Linux) would silently never arrive, which would make this
        // module untestable/unverifiable without a real LAN. Binding to
        // INADDR_ANY still receives genuine LAN broadcasts addressed to this
        // host, so real GameController traffic works identically either way.
        // Binding can throw if the port is already taken (another sim instance, or a
        // real GameController listener on this host). That must not crash the whole
        // sim — degrade to "no supervisor" and log, matching the no-GC idle contract.
        try {
            on<UDP, Single>(gc_port).then([this](const UDP::Packet& packet) {
                gc::GameControllerPacket parsed{};
                if (!gc::try_parse(packet.payload.data(), packet.payload.size(), parsed)) {
                    // Not a (recognised-version) GameController packet — could be
                    // unrelated traffic on this port, or a header/version mismatch.
                    // Silently ignored, per "no GC on the network" being a normal,
                    // error-free idle state, not a fault.
                    return;
                }

                const mjModel* m      = model_.load(std::memory_order_acquire);
                mjData* d             = data_.load(std::memory_order_acquire);
                std::mutex* sim_mutex = sim_mutex_.load(std::memory_order_acquire);
                if (m == nullptr || d == nullptr || sim_mutex == nullptr) {
                    log<NUClear::LogLevel::WARN>(
                        "GameController packet received before the sim model was ready — ignoring");
                    return;
                }

                if (last_state_ >= 0 && last_state_ != int(gc::State::INITIAL) && parsed.state == gc::State::INITIAL
                    && reset_world_) {
                    reset_world_();
                    log<NUClear::LogLevel::INFO>("Supervisor: match reset -> startup poses and controllers");
                }
                last_state_ = int(parsed.state);
                std::vector<SupervisorLogic::Action> actions;
                {
                    std::lock_guard<std::mutex> lock(*sim_mutex);
                    actions = logic_->process(m, d, parsed);
                    for (const auto& action : actions) {
                        if (action.robot_id && placed_robot_)
                            placed_robot_(action.robot_id);
                    }
                    mj_forward(m, d);
                }
                for (const auto& action : actions) {
                    if (action.level == SupervisorLogic::Action::Level::WARN) {
                        log<NUClear::LogLevel::WARN>(action.message.c_str());
                    }
                    else {
                        log<NUClear::LogLevel::INFO>(action.message.c_str());
                    }
                }
            });
        }
        catch (const std::exception& e) {
            log<NUClear::LogLevel::WARN>("Supervisor: could not bind UDP port",
                                         gc_port,
                                         "(",
                                         e.what(),
                                         ") — running without GameController placement");
        }

        on<Startup>().then([this, gc_port] {
            log<NUClear::LogLevel::INFO>("Supervisor ready — waiting for GameController broadcasts on port", gc_port);
        });

        on<Shutdown>().then([this] { log<NUClear::LogLevel::INFO>("Supervisor shutting down"); });
    }

}  // namespace k1sim::module
