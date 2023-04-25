//
// Omnidirectional velocity control in body coordinates (forward-right-down)
//


#include <chrono>
#include <cmath>
#include <future>
#include <iostream>
#include <thread>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

using namespace mavsdk;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;



// ======================================================================
void usage(const std::string& bin_name)
{
    std::cerr << "Usage : " << bin_name << " <connection_url>\n"
              << "Connection URL format should be :\n"
              << " For TCP : tcp://[server_host][:server_port]\n"
              << " For UDP : udp://[bind_host][:bind_port]\n"
              << " For Serial : serial:///path/to/serial/dev[:baudrate]\n"
              << "For example, to connect to the simulator use URL: udp://:14540\n";
}

std::shared_ptr<System> get_system(Mavsdk& mavsdk)
{
    std::cout << "Waiting to discover system...\n";
    auto prom = std::promise<std::shared_ptr<System>>{};
    auto fut = prom.get_future();

    // Wait for new systems to be discovered, use the autopilot once found
    mavsdk.subscribe_on_new_system([&mavsdk, &prom]() {
        auto system = mavsdk.systems().back();

        if (system->has_autopilot()) {
            std::cout << "Discovered autopilot\n";

            // Unsubscribe again to get only one system
            mavsdk.subscribe_on_new_system(nullptr);
            prom.set_value(system);
        }
    });

    // Wait at least 3 seconds to find an autopilot system
    if (fut.wait_for(seconds(3)) == std::future_status::timeout) {
        std::cerr << "No autopilot found.\n";
        return {};
    }

    // Get discovered system now
    return fut.get();
}


// ======================================================================
// Does Offboard control using body coordinates
// Returns true if everything went well in Offboard control
bool offb_ctrl_body(mavsdk::Offboard& offboard)
{
    std::cout << "Starting Offboard velocity control in body coordinates\n";

    // Send it once before starting offboard, otherwise it will be rejected
    Offboard::VelocityBodyYawspeed hover{};
    offboard.set_velocity_body(hover);

    // Start Offboard mode
    Offboard::Result offboard_result = offboard.start();
    if (offboard_result != Offboard::Result::Success) {
        std::cerr << "Offboard start failed: " << offboard_result << '\n';
        return false;
    }
    std::cout << "Offboard started\n";

    // ----------------------------------------
    // Hover
    Offboard::VelocityBodyYawspeed hover{};
    std::cout << "Hover\n";
    offboard.set_velocity_body(hover);
    sleep_for(seconds(2));

    // ----------------------------------------
    // Fly forward, right, up
    std::cout << "Fly forward, right, up\n";
    Offboard::VelocityBodyYawspeed fly_forward_right_up{};
    fly_forward_right_up.forward_m_s = 0.5f;
    fly_forward_right_up.right_m_s = 0.5f;
    fly_forward_right_up.down_m_s = -0.25f;
    offboard.set_velocity_body(fly_forward_right_up);
    sleep_for(seconds(4));

    // Fly forward, left, down
    std::cout << "Fly forward, left, down\n";
    Offboard::VelocityBodyYawspeed fly_forward_left_down{};
    fly_forward_left_down.forward_m_s = 0.5f;
    fly_forward_left_down.right_m_s = -0.5f;
    fly_forward_left_down.down_m_s = 0.25f;
    offboard.set_velocity_body(fly_forward_left_down);
    sleep_for(seconds(4));

    // Fly backward, left, up
    std::cout << "Fly backward, left, up\n";
    Offboard::VelocityBodyYawspeed fly_backward_left_up{};
    fly_backward_left_up.forward_m_s = -0.5f;
    fly_backward_left_up.right_m_s = -0.5f;
    fly_backward_left_up.down_m_s = -0.25f;
    offboard.set_velocity_body(fly_backward_left_up);
    sleep_for(seconds(4));

    // Fly backward, right, down
    std::cout << "Fly backward, right, down\n";
    Offboard::VelocityBodyYawspeed fly_backward_right_down{};
    fly_backward_right_down.forward_m_s = -0.5f;
    fly_backward_right_down.right_m_s = 0.5f;
    fly_backward_right_down.down_m_s = 0.25f;
    offboard.set_velocity_body(fly_backward_right_down);
    sleep_for(seconds(4));

    // ----------------------------------------
    // Fly quarter circle up
    std::cout << "Fly quarter circle up\n";
    Offboard::VelocityBodyYawspeed fly_quarter_cirle_up{};
    fly_quarter_cirle_up.right_m_s = 0.5f;
    fly_quarter_cirle_up.down_m_s = -0.25f;
    fly_quarter_cirle_up.yawspeed_deg_s = 22.5f;
    offboard.set_velocity_body(fly_quarter_cirle_up);
    sleep_for(seconds(4));

    // Fly quarter circle down
    std::cout << "Fly quarter circle down\n";
    Offboard::VelocityBodyYawspeed fly_quarter_cirle_down{};
    fly_quarter_cirle_down.right_m_s = 0.5f;
    fly_quarter_cirle_down.down_m_s = 0.25f;
    fly_quarter_cirle_down.yawspeed_deg_s = 22.5f;
    offboard.set_velocity_body(fly_quarter_cirle_down);
    sleep_for(seconds(4));

    // Fly quarter circle up
    std::cout << "Fly quarter circle up\n";
    Offboard::VelocityBodyYawspeed fly_quarter_cirle_up{};
    fly_quarter_cirle_up.right_m_s = 0.5f;
    fly_quarter_cirle_up.down_m_s = -0.25f;
    fly_quarter_cirle_up.yawspeed_deg_s = 22.5f;
    offboard.set_velocity_body(fly_quarter_cirle_up);
    sleep_for(seconds(4));

    // Fly quarter circle down
    std::cout << "Fly quarter circle down\n";
    Offboard::VelocityBodyYawspeed fly_quarter_cirle_down{};
    fly_quarter_cirle_down.right_m_s = 0.5f;
    fly_quarter_cirle_down.down_m_s = 0.25f;
    fly_quarter_cirle_down.yawspeed_deg_s = 22.5f;
    offboard.set_velocity_body(fly_quarter_cirle_down);
    sleep_for(seconds(4));

    // ----------------------------------------
    // Hover
    Offboard::VelocityBodyYawspeed hover{};
    std::cout << "Hover\n";
    offboard.set_velocity_body(hover);
    sleep_for(seconds(2));

    // ----------------------------------------
    // Stop Offboard mode
    offboard_result = offboard.stop();
    if (offboard_result != Offboard::Result::Success) {
        std::cerr << "Offboard stop failed: " << offboard_result << '\n';
        return false;
    }
    std::cout << "Offboard stopped\n";

    return true;
}


// ======================================================================
// Main function
int main(int argc, char** argv)
{
    if (argc != 2) {
        usage(argv[0]);
        return 1;
    }

    // Add connection
    Mavsdk mavsdk;
    ConnectionResult connection_result = mavsdk.add_any_connection(argv[1]);

    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result << '\n';
        return 1;
    }

    auto system = get_system(mavsdk);
    if (!system) {
        return 1;
    }

    // Instantiate plugins.
    auto action = Action{system};
    auto offboard = Offboard{system};
    auto telemetry = Telemetry{system};

    // Check until vehicle is ready to arm
    while (!telemetry.health_all_ok()) {
        std::cout << "Waiting for system to be ready\n";
        sleep_for(seconds(1));
    }
    std::cout << "System is ready\n";

    // Arm vehicle
    const auto arm_result = action.arm();
    if (arm_result != Action::Result::Success) {
        std::cerr << "Arming failed: " << arm_result << '\n';
        return 1;
    }
    std::cout << "Armed\n";

    // Take off
    action.set_takeoff_altitude(1.5);
    action.set_current_speed(0.25);
    const auto takeoff_result = action.takeoff();
    if (takeoff_result != Action::Result::Success) {
        std::cerr << "Takeoff failed: " << takeoff_result << '\n';
        return 1;
    }

    // Check if the vehicle is in the air after takeoff
    auto in_air_promise = std::promise<void>{};
    auto in_air_future = in_air_promise.get_future();
    telemetry.subscribe_landed_state([&telemetry, &in_air_promise](Telemetry::LandedState state) {
        if (state == Telemetry::LandedState::InAir) {
            std::cout << "Taking off has finished\n.";
            telemetry.subscribe_landed_state(nullptr);
            in_air_promise.set_value();
        }
    });
    in_air_future.wait_for(seconds(10));
    if (in_air_future.wait_for(seconds(3)) == std::future_status::timeout) {
        std::cerr << "Takeoff timed out.\n";
        return 1;
    }

    //  Omnidirectional velocity control with body coordinates
    if (!offb_ctrl_body(offboard)) {
        return 1;
    }

    // Landing
    const auto land_result = action.land();
    if (land_result != Action::Result::Success) {
        std::cerr << "Landing failed: " << land_result << '\n';
        return 1;
    }

    // Check if the vehicle is still in the air while landing
    while (telemetry.in_air()) {
        std::cout << "Vehicle is landing...\n";
        sleep_for(seconds(1));
    }
    std::cout << "Landed!\n";

    // Wait to ensure safety and auto-disarm
    sleep_for(seconds(3));
    std::cout << "Finished...\n";

    return 0;
}