from setuptools import setup

package_name = "pinky_control_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (
            f"share/{package_name}/scripts",
            [
                "scripts/robot_bringup.sh",
                "scripts/robot_session.sh",
                "scripts/install.sh",
            ],
        ),
        (f"share/{package_name}/scripts/lib", ["scripts/lib/wait_for.sh"]),
        (
            f"share/{package_name}/systemd",
            [
                "systemd/pinky-bringup@.service",
                "systemd/pinky-session@.service",
            ],
        ),
        (
            f"share/{package_name}/config",
            ["config/robot_1.env", "config/robot_2.env"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
)
