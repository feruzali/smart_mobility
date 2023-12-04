# TurtleBot3 SBC Setup Report

## Introduction

The TurtleBot3 is a widely used robot platform for educational and research purposes. This setup report guides you through the essential steps of configuring the Single Board Computer (SBC) for your TurtleBot3. This process involves installing Ubuntu Server 22.04, ROS2 Humble Hawksbill, and the required packages.

## Procedure

### 1. Network Configuration

Open the network configuration file:

```bash
sudo nano /writable/etc/netplan/50-cloud-init.yaml
```

Edit content with your WiFi SSID and password. Save and exit.

### 2. Update Automatic Update Settings

Edit update settings file:

```bash
sudo nano /etc/apt/apt.conf.d/20auto-upgrades
```

Set update settings. Save and exit.

### 3. System Configuration

Prevent boot-up delay without network:

```bash
systemctl mask systemd-networkd-wait-online.service
```

### 4. Disable Suspend and Hibernation

Disable sleep-related targets:

```bash
sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target
```

### 5. Reboot Raspberry Pi

```bash
reboot
```

### 6. SSH Access

After reboot, SSH into Raspberry Pi from the remote PC:

```bash
ssh turtle@{IP Address of Raspberry PI}
```

### 7. USB Port Setting for OpenCR

Copy USB rules for OpenCR:

```bash
sudo cp ros2 pkg prefix turtlebot3_bringup/share/turtlebot3_bringup/script/99-turtlebot3-cdc.rules /etc/udev/rules.d/
```

Reload rules:

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 8. ROS Domain ID Setting

Set ROS_DOMAIN_ID to 30:

```bash
echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
```

### 9. LDS Configuration

Set LDS model based on the year of purchase:

```bash
echo 'export LDS_MODEL=LDS-02' >> ~/.bashrc
```

### 10. Apply Changes

Apply changes to the bashrc:

```bash
source ~/.bashrc
```

## Conclusion

Following these steps ensures the successful setup of the TurtleBot3 SBC. It includes the installation of the operating system, ROS2 Humble Hawksbill, and the necessary packages for the TurtleBot3 robot. The configuration steps enhance security, optimize user experience, and set up communication parameters for effective robot operation.
