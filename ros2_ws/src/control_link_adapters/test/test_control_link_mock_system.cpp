#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "control_link_adapters/control_link_mock_system.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace
{
	using CallbackReturn = control_link_adapters::ControlLinkMockSystem::CallbackReturn;

	hardware_interface::InterfaceInfo interface(std::string name)
	{
		hardware_interface::InterfaceInfo result;
		result.name = std::move(name);
		return result;
	}

	hardware_interface::ComponentInfo wheel(std::string name)
	{
		hardware_interface::ComponentInfo result;
		result.name = std::move(name);
		result.type = "joint";
		result.command_interfaces = {interface(hardware_interface::HW_IF_VELOCITY)};
		result.state_interfaces = {
			interface(hardware_interface::HW_IF_POSITION),
			interface(hardware_interface::HW_IF_VELOCITY)};
		return result;
	}

	hardware_interface::HardwareInfo valid_info()
	{
		hardware_interface::HardwareInfo result;
		result.name = "ControlLinkMockSystem";
		result.type = "system";
		result.hardware_class_type = "control_link_adapters/ControlLinkMockSystem";
		result.hardware_parameters = {
			{"left_wheel_joint", "left_wheel_joint"},
			{"right_wheel_joint", "right_wheel_joint"},
			{"fail_read_after_cycles", "0"},
			{"fail_write_after_cycles", "0"}};
		result.joints = {wheel("left_wheel_joint"), wheel("right_wheel_joint")};
		return result;
	}

	void configure_and_activate(control_link_adapters::ControlLinkMockSystem & system)
	{
		const rclcpp_lifecycle::State previous_state;
		ASSERT_EQ(system.on_configure(previous_state), CallbackReturn::SUCCESS);
		ASSERT_EQ(system.on_activate(previous_state), CallbackReturn::SUCCESS);
	}
}  // namespace

TEST(ControlLinkMockSystem, ExportsConfiguredWheelInterfacesAndIntegratesPeriod)
{
	control_link_adapters::ControlLinkMockSystem system;
	ASSERT_EQ(system.on_init(valid_info()), CallbackReturn::SUCCESS);

	auto state_interfaces = system.export_state_interfaces();
	auto command_interfaces = system.export_command_interfaces();
	ASSERT_EQ(state_interfaces.size(), 4U);
	ASSERT_EQ(command_interfaces.size(), 2U);
	EXPECT_EQ(command_interfaces[0].get_name(), "left_wheel_joint/velocity");
	EXPECT_EQ(command_interfaces[1].get_name(), "right_wheel_joint/velocity");
	EXPECT_EQ(state_interfaces[0].get_name(), "left_wheel_joint/position");
	EXPECT_EQ(state_interfaces[1].get_name(), "left_wheel_joint/velocity");

	configure_and_activate(system);
	command_interfaces[0].set_value(2.0);
	command_interfaces[1].set_value(-1.0);
	const rclcpp::Time now(0, 0, RCL_ROS_TIME);
	const auto period = rclcpp::Duration::from_seconds(0.25);
	ASSERT_EQ(system.write(now, period), hardware_interface::return_type::OK);
	ASSERT_EQ(system.read(now, period), hardware_interface::return_type::OK);
	EXPECT_DOUBLE_EQ(state_interfaces[0].get_value(), 0.5);
	EXPECT_DOUBLE_EQ(state_interfaces[1].get_value(), 2.0);
	EXPECT_DOUBLE_EQ(state_interfaces[2].get_value(), -0.25);
	EXPECT_DOUBLE_EQ(state_interfaces[3].get_value(), -1.0);
}

TEST(ControlLinkMockSystem, RejectsReadWriteOutsideActiveAndClearsMotionOnDeactivate)
{
	control_link_adapters::ControlLinkMockSystem system;
	ASSERT_EQ(system.on_init(valid_info()), CallbackReturn::SUCCESS);
	auto states = system.export_state_interfaces();
	auto commands = system.export_command_interfaces();
	const rclcpp::Time now(0, 0, RCL_ROS_TIME);
	const auto period = rclcpp::Duration::from_seconds(0.1);

	EXPECT_EQ(system.read(now, period), hardware_interface::return_type::ERROR);
	EXPECT_EQ(system.write(now, period), hardware_interface::return_type::ERROR);
	configure_and_activate(system);
	commands[0].set_value(1.0);
	ASSERT_EQ(system.write(now, period), hardware_interface::return_type::OK);
	ASSERT_EQ(system.read(now, period), hardware_interface::return_type::OK);
	EXPECT_GT(states[0].get_value(), 0.0);

	const rclcpp_lifecycle::State previous_state;
	ASSERT_EQ(system.on_deactivate(previous_state), CallbackReturn::SUCCESS);
	EXPECT_DOUBLE_EQ(states[0].get_value(), 0.0);
	EXPECT_DOUBLE_EQ(states[1].get_value(), 0.0);
	EXPECT_EQ(system.write(now, period), hardware_interface::return_type::ERROR);
}

TEST(ControlLinkMockSystem, RejectsNonFiniteCommandsWithoutUpdatingVelocity)
{
	control_link_adapters::ControlLinkMockSystem system;
	ASSERT_EQ(system.on_init(valid_info()), CallbackReturn::SUCCESS);
	auto states = system.export_state_interfaces();
	auto commands = system.export_command_interfaces();
	configure_and_activate(system);

	commands[0].set_value(std::numeric_limits<double>::quiet_NaN());
	commands[1].set_value(0.0);
	EXPECT_EQ(
		system.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.02)),
		hardware_interface::return_type::ERROR);
	EXPECT_DOUBLE_EQ(states[1].get_value(), 0.0);
}

TEST(ControlLinkMockSystem, InjectsReadAndWriteFailuresAfterSuccessfulCycles)
{
	auto read_info = valid_info();
	read_info.hardware_parameters["fail_read_after_cycles"] = "1";
	control_link_adapters::ControlLinkMockSystem read_system;
	ASSERT_EQ(read_system.on_init(read_info), CallbackReturn::SUCCESS);
	configure_and_activate(read_system);
	EXPECT_EQ(
		read_system.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.02)),
		hardware_interface::return_type::OK);
	EXPECT_EQ(
		read_system.read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.02)),
		hardware_interface::return_type::ERROR);

	auto write_info = valid_info();
	write_info.hardware_parameters["fail_write_after_cycles"] = "1";
	control_link_adapters::ControlLinkMockSystem write_system;
	ASSERT_EQ(write_system.on_init(write_info), CallbackReturn::SUCCESS);
	configure_and_activate(write_system);
	EXPECT_EQ(
		write_system.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.0)),
		hardware_interface::return_type::OK);
	EXPECT_EQ(
		write_system.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.0)),
		hardware_interface::return_type::ERROR);
}

TEST(ControlLinkMockSystem, RejectsMalformedHardwareDefinitions)
{
	std::vector<hardware_interface::HardwareInfo> invalid;

	auto missing_joint_parameter = valid_info();
	missing_joint_parameter.hardware_parameters.erase("left_wheel_joint");
	invalid.push_back(std::move(missing_joint_parameter));

	auto duplicate_joint_names = valid_info();
	duplicate_joint_names.hardware_parameters["right_wheel_joint"] = "left_wheel_joint";
	invalid.push_back(std::move(duplicate_joint_names));

	auto wrong_command_interface = valid_info();
	wrong_command_interface.joints[0].command_interfaces = {
		interface(hardware_interface::HW_IF_POSITION)};
	invalid.push_back(std::move(wrong_command_interface));

	auto duplicate_state_interface = valid_info();
	duplicate_state_interface.joints[0].state_interfaces.push_back(
		interface(hardware_interface::HW_IF_POSITION));
	invalid.push_back(std::move(duplicate_state_interface));

	auto malformed_threshold = valid_info();
	malformed_threshold.hardware_parameters["fail_read_after_cycles"] = "1cycle";
	invalid.push_back(std::move(malformed_threshold));

	for (const auto & info : invalid)
	{
		control_link_adapters::ControlLinkMockSystem system;
		EXPECT_EQ(system.on_init(info), CallbackReturn::ERROR);
	}
}

TEST(ControlLinkMockSystem, PluginlibLoadsSystemInterface)
{
	pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
		"hardware_interface", "hardware_interface::SystemInterface");
	EXPECT_TRUE(loader.isClassAvailable("control_link_adapters/ControlLinkMockSystem"));
	auto system = loader.createSharedInstance("control_link_adapters/ControlLinkMockSystem");
	ASSERT_NE(system, nullptr);
	EXPECT_EQ(system->on_init(valid_info()), CallbackReturn::SUCCESS);
}
