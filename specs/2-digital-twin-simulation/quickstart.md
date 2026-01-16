# Quickstart Guide: Digital Twin Simulation for Humanoid Robotics

## Overview
This guide provides a quick introduction to getting started with Module 2 of "Physical AI & Humanoid Robotics" focused on digital twin simulation for humanoid robots. This includes setting up Docusaurus and creating four simulation-focused chapters with Docusaurus-compatible Markdown files organized for easy navigation.

## Prerequisites

### System Requirements
- Operating System: Ubuntu 20.04/22.04, Windows 10/11, or macOS 10.15+
- RAM: Minimum 8GB (16GB recommended)
- Storage: 10GB available space
- Graphics: OpenGL 3.3 compatible GPU

### Software Dependencies
1. **Node.js** (v16.14 or higher) - Required for Docusaurus
   ```bash
   node --version
   npm --version
   ```

2. **ROS 2 Installation** (Humble Hawksbill or newer)
   ```bash
   # Follow official ROS 2 installation guide for your OS
   # https://docs.ros.org/en/humble/Installation.html
   ```

3. **Gazebo Garden** (for physics simulation)
   ```bash
   # Installation instructions vary by OS
   sudo apt install gazebo
   ```

4. **Unity Hub** (for 3D environment creation)
   - Download from: https://unity.com/download

5. **Python 3.8+** (for scripting and tools)
   ```bash
   python3 --version
   ```

6. **Git** for version control
   ```bash
   git --version
   ```

## Setting up Docusaurus Documentation

### 1. Initialize Docusaurus Project
```bash
# Navigate to the book frontend directory
cd book_frontend

# If the directory doesn't exist, create it
mkdir -p book_frontend && cd book_frontend

# Initialize Docusaurus if not already done
npx create-docusaurus@latest website classic

# Or if you're adding to an existing project, install Docusaurus
npm init
npm install @docusaurus/core @docusaurus/preset-classic
```

### 2. Install Additional Dependencies
```bash
# Navigate to the book frontend directory
cd book_frontend

# Install Node.js dependencies
npm install
```

### 3. Create Module 2 Documentation Structure
```bash
# Create the module 2 documentation directory
mkdir -p docs/module2

# Create the four required chapters
touch docs/module2/digital-twins-fundamentals.md
touch docs/module2/physics-simulation-with-gazebo.md
touch docs/module2/unity-environment-interaction.md
touch docs/module2/sensor-simulation-humanoids.md
```

### 4. Update Docusaurus Configuration
Update `docusaurus.config.js` and `sidebars.js` to include the new module:

`docusaurus.config.js`:
```javascript
// Add to presets configuration
presets: [
  [
    '@docusaurus/preset-classic',
    {
      docs: {
        sidebarPath: require.resolve('./sidebars.js'),
        // Add routeBasePath for docs to serve from /
      },
      theme: {
        customCss: require.resolve('./src/css/custom.css'),
      },
    },
  ],
],
```

`sidebars.js`:
```javascript
module.exports = {
  docs: [
    {
      type: 'category',
      label: 'Module 2: Digital Twin Simulation',
      items: [
        'module2/digital-twins-fundamentals',
        'module2/physics-simulation-with-gazebo',
        'module2/unity-environment-interaction',
        'module2/sensor-simulation-humanoids',
      ],
    },
  ],
};
```

## Running the Documentation Server

### 1. Start Docusaurus Development Server
```bash
cd book_frontend
npm start
```

### 2. Access Documentation
Open your browser to `http://localhost:3000` to view the module documentation.

## Setting Up Simulation Environment

### 1. Verify ROS 2 Installation
```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash  # Adjust for your ROS 2 distribution

# Verify installation
ros2 topic list
```

### 2. Set Up Simulation Workspace
```bash
# Create a workspace for simulation packages
mkdir -p ~/simulation_ws/src
cd ~/simulation_ws

# Build the workspace
colcon build
source install/setup.bash
```

## First Simulation

### 1. Run a Basic Gazebo Simulation
```bash
# Launch a simple humanoid robot simulation
ros2 launch example_simulation simple_humanoid.launch.py
```

### 2. Interact with the Simulation
- Use the Gazebo GUI to pause/resume the simulation
- Monitor robot topics using `ros2 topic echo`
- Send commands to the robot using `ros2 topic pub`

### 3. Explore the Digital Twin Concepts
- Review the documentation in `book_frontend/docs/module2/`
- Complete the first hands-on exercise
- Run the provided sensor simulation examples

## Key Directories and Files

```
book_frontend/
├── docs/
│   └── module2/                 # Module 2 documentation
│       ├── digital-twins-fundamentals.md
│       ├── physics-simulation-with-gazebo.md
│       ├── unity-environment-interaction.md
│       └── sensor-simulation-humanoids.md
├── tutorial/
│   └── simulation-examples/     # Practical examples
│       ├── gazebo-scenarios/
│       ├── unity-scenes/
│       └── sensor-simulations/
├── src/
│   ├── components/              # Custom Docusaurus components
│   └── pages/                   # Additional pages
├── docusaurus.config.js         # Docusaurus configuration
├── sidebars.js                  # Navigation sidebar configuration
├── package.json                 # Node.js dependencies
└── static/                      # Static assets
```

## Common Commands

### Docusaurus Commands
```bash
# Start development server
npm start

# Build static files
npm run build

# Serve built files locally
npm run serve

# Clear cache
npm run clear
```

### ROS 2 Commands
```bash
# List all topics
ros2 topic list

# Echo a specific topic
ros2 topic echo /robot/joint_states

# List all nodes
ros2 node list

# Get information about services
ros2 service list
```

### Gazebo Commands
```bash
# Launch Gazebo with a specific world
gazebo worlds/empty.world

# Launch with GUI disabled (for headless operation)
gz sim -s server_config.yaml
```

## Troubleshooting

### Common Issues

#### 1. Docusaurus Not Installing Properly
**Solution**: Clear npm cache and reinstall:
```bash
npm cache clean --force
rm -rf node_modules package-lock.json
npm install
```

#### 2. ROS 2 Environment Not Found
**Solution**: Source your ROS 2 installation:
```bash
source /opt/ros/humble/setup.bash
# Or add to your ~/.bashrc file permanently
```

#### 3. Gazebo Not Starting
**Solution**: Check graphics drivers and ensure X11 forwarding if using SSH:
```bash
echo $DISPLAY  # Should show display information
```

#### 4. Python Package Issues
**Solution**: Create a virtual environment and install requirements:
```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

### Getting Help
- Check the FAQ in the documentation
- Use `ros2 <command> --help` for command-specific help
- Consult the troubleshooting guide in `book_frontend/docs/troubleshooting.md`

## Next Steps

1. Complete the Digital Twins fundamentals chapter
2. Work through the Physics Simulation with Gazebo tutorial
3. Explore Unity Environment Interaction
4. Experiment with Sensor Simulation for Humanoids
5. Attempt the hands-on exercises in each section

## Resources

- [Docusaurus Documentation](https://docusaurus.io/docs)
- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [Gazebo Documentation](https://gazebosim.org/docs)
- [Unity Learn](https://learn.unity.com/)
- [Module 2 Assessment Rubric](link-to-assessment-rubric)