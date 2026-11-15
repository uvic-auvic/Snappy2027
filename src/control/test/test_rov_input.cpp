#include "rov_input.hpp"
#include <gtest/gtest.h>
#include <limits>

class RovInputTest : public ::testing::Test {
protected:
    RovConfig config() { RovConfig c; c.max_depth = 5.0; return c; }
    RovInput input{config()};
    std::vector<float> axes{0, 0, 1, 0, 0, 1, 0, 0};
    std::vector<int32_t> buttons = std::vector<int32_t>(13, 0);
    double time = 1.0;
    void send() { ASSERT_TRUE(input.receive(axes, buttons, time)); time += 0.01; }
    RovRequest output() { return input.request(time); }
    void arm() {
        input.update_depth(2.0, true);
        send();
        buttons[4] = 1;
        send();
    }
};

TEST_F(RovInputTest, StartupHeldDeadmanCannotArm) {
    input.update_depth(2, true);
    buttons[4] = 1; axes[5] = -1; send();
    EXPECT_FALSE(output().manual_enabled);
    buttons[4] = 0; send(); buttons[4] = 1; send();
    EXPECT_DOUBLE_EQ(output().surge, 0.3);
}
TEST_F(RovInputTest, TriggerEffortIsAnalogAndSubtracts) {
    arm(); axes[5] = 0; send(); EXPECT_DOUBLE_EQ(output().surge, 0.15);
    axes[5] = 1; axes[2] = -1; send(); EXPECT_DOUBLE_EQ(output().surge, -0.3);
    axes[5] = -0.6f; axes[2] = 0.4f; send(); EXPECT_NEAR(output().surge, 0.15, 1e-7);
    axes[5] = -1; axes[2] = -1; send(); EXPECT_DOUBLE_EQ(output().surge, 0);
}
TEST_F(RovInputTest, SticksHaveInversionDeadzoneAndLimits) {
    arm(); axes[0] = 0.05; axes[3] = -0.05; send();
    EXPECT_DOUBLE_EQ(output().sway, 0); EXPECT_DOUBLE_EQ(output().yaw, 0);
    axes[0] = -1; axes[3] = 1; send();
    EXPECT_DOUBLE_EQ(output().sway, 0.3); EXPECT_DOUBLE_EQ(output().yaw, -0.3);
    axes[1] = -1; axes[4] = 1; send();  // Vertical sticks have no effect.
    EXPECT_DOUBLE_EQ(output().sway, 0.3); EXPECT_DOUBLE_EQ(output().yaw, -0.3);
}
TEST_F(RovInputTest, DepthStepsOnceAndRequiresDpadRelease) {
    arm(); EXPECT_DOUBLE_EQ(output().depth_target, 2);
    axes[7] = -1; send(); send(); send(); EXPECT_DOUBLE_EQ(output().depth_target, 2.25);
    axes[7] = 0; send(); axes[7] = -1; send(); EXPECT_DOUBLE_EQ(output().depth_target, 2.5);
    axes[7] = 0; send(); axes[7] = 1; send(); EXPECT_DOUBLE_EQ(output().depth_target, 2.25);
}
TEST_F(RovInputTest, DeadmanReleaseKeepsDepthAndIgnoresDpad) {
    arm(); axes[5] = -1; axes[0] = 1; axes[3] = 1; send();
    buttons[4] = 0; axes[7] = -1; send();
    EXPECT_DOUBLE_EQ(output().surge, 0); EXPECT_DOUBLE_EQ(output().sway, 0);
    EXPECT_DOUBLE_EQ(output().yaw, 0); EXPECT_TRUE(output().depth_enabled);
    EXPECT_DOUBLE_EQ(output().depth_target, 2);
    buttons[4] = 1; send(); EXPECT_DOUBLE_EQ(output().depth_target, 2);
}
TEST_F(RovInputTest, DepthLimitsAndSurfaceGuard) {
    arm(); input.update_depth(0.02, true);
    axes[7] = 1; send(); EXPECT_DOUBLE_EQ(output().depth_target, 2);  // No UP near surface.
    input.update_depth(2, true);
    for (int i = 0; i < 30; ++i) { axes[7] = 0; send(); axes[7] = 1; send(); }
    EXPECT_DOUBLE_EQ(output().depth_target, 0);
    for (int i = 0; i < 30; ++i) { axes[7] = 0; send(); axes[7] = -1; send(); }
    EXPECT_DOUBLE_EQ(output().depth_target, 5);
}
TEST_F(RovInputTest, TimeoutRequiresNewReleaseEvenBeforeTimerTick) {
    arm(); axes[5] = -1; send(); time += 0.6; send();
    EXPECT_FALSE(output().manual_enabled); EXPECT_TRUE(output().depth_enabled);
    buttons[4] = 0; send(); buttons[4] = 1; send(); EXPECT_DOUBLE_EQ(output().surge, 0.3);
    time += 0.6; EXPECT_DOUBLE_EQ(output().surge, 0);
}
TEST_F(RovInputTest, FaultRecapturesDepthAndDisarms) {
    arm(); input.update_depth(2, false); EXPECT_FALSE(output().depth_enabled);
    input.update_depth(3, true); send(); EXPECT_FALSE(output().manual_enabled);
    EXPECT_DOUBLE_EQ(output().depth_target, 3);
}
TEST_F(RovInputTest, StopIsLocalAndLatchedByControllerUntilReset) {
    arm(); buttons[1] = 1; send(); EXPECT_TRUE(output().emergency_stop);
    EXPECT_FALSE(output().depth_enabled);
    buttons[1] = 0; send(); EXPECT_FALSE(output().depth_enabled);
    input.set_stop_latched(false); send(); EXPECT_FALSE(output().depth_enabled);
    input.set_stop_latched(true); input.set_stop_latched(false);
    input.update_depth(2, true); send(); EXPECT_FALSE(output().manual_enabled);
    buttons[4] = 0; send(); buttons[4] = 1; send(); EXPECT_TRUE(output().manual_enabled);
}
TEST_F(RovInputTest, MalformedMessagesDisarmButStillHonorCircle) {
    arm(); auto short_axes = std::vector<float>{}; buttons[1] = 1;
    EXPECT_FALSE(input.receive(short_axes, buttons, time));
    EXPECT_TRUE(output().emergency_stop); EXPECT_FALSE(output().depth_enabled);
}
TEST_F(RovInputTest, RejectsNonfiniteAndOutOfRangeAndBadButtons) {
    arm(); axes[0] = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(input.receive(axes, buttons, time)); EXPECT_FALSE(output().manual_enabled);
    axes[0] = 2; EXPECT_FALSE(input.receive(axes, buttons, time));
    axes[0] = 0; buttons[0] = -1; EXPECT_FALSE(input.receive(axes, buttons, time));
    EXPECT_FALSE(input.receive(axes, {}, time));
}
TEST(RovConfigTest, ValidatesSafetyParameters) {
    RovConfig c; EXPECT_NO_THROW(c.validate()); EXPECT_FALSE(c.depth_limits_configured());
    c.max_depth = 0; EXPECT_THROW(c.validate(), std::invalid_argument);
    c.max_depth = 5; c.deadzone = 1; EXPECT_THROW(c.validate(), std::invalid_argument);
    c.deadzone = 0.1; c.joy_timeout = 0; EXPECT_THROW(c.validate(), std::invalid_argument);
    c.joy_timeout = 0.5; c.trigger_pressed = c.trigger_released;
    EXPECT_THROW(c.validate(), std::invalid_argument);
}
TEST(RovConfigTest, AlternativeMappingAndTriggerNormalization) {
    RovConfig c; c.max_depth = 5; c.trigger_released = 0; c.trigger_pressed = 1;
    c.invert_strafe = false; c.invert_yaw = false; c.dpad_axis = -1;
    c.dpad_up_button = 10; c.dpad_down_button = 11;
    RovInput input(c); input.update_depth(2, true);
    std::vector<float> axes(6, 0); std::vector<int32_t> buttons(12, 0);
    input.receive(axes, buttons, 1); buttons[4] = 1; axes[5] = 0.5;
    axes[0] = 1; axes[3] = 1; buttons[11] = 1; input.receive(axes, buttons, 1.1);
    const auto out = input.request(1.1);
    EXPECT_DOUBLE_EQ(out.surge, 0.15); EXPECT_DOUBLE_EQ(out.sway, 0.3);
    EXPECT_DOUBLE_EQ(out.yaw, 0.3); EXPECT_DOUBLE_EQ(out.depth_target, 2.25);
}
