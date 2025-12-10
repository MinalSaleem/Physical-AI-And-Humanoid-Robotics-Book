import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Physical AI and Humanoid Robotics',
      link: {type: 'doc', id: 'intro'},
      items: [
        {
          type: 'category',
          label: 'Module 1: The Robotic Nervous System (ROS 2)',
          link: {type: 'doc', id: 'module-one/01-ros2-middleware-intro'},
          items: [
            'module-one/01-ros2-middleware-intro',
            'module-one/02-nodes-topics-services',
            'module-one/03-python-rclpy-integration',
            'module-one/04-urdf-humanoids',
            'module-one/05-module-one-glossary',
          ],
        },
        {
          type: 'category',
          label: 'Module 2: The Digital Twin (Gazebo & Unity)',
          link: {type: 'doc', id: 'module-two/01-physics-sim-env-building'},
          items: [
            'module-two/01-physics-sim-env-building',
            'module-two/02-gazebo-physics-collisions',
            'module-two/03-unity-rendering-hri',
            'module-two/04-simulating-sensors',
            'module-two/05-module-two-glossary',
          ],
        },
        {
          type: 'category',
          label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
          link: {type: 'doc', id: 'module-three/01-advanced-perception-training'},
          items: [
            'module-three/01-advanced-perception-training',
            'module-three/02-nvidia-isaac-sim',
            'module-three/03-isaac-ros-vslam-navigation',
            'module-three/04-nav2-humanoid-path-planning',
          ],
        },
        {
          type: 'category',
          label: 'Module 4: Vision-Language-Action (VLA)',
          link: {type: 'doc', id: '04-module-four/01-llms-robotics-convergence'},
          items: [
            '04-module-four/01-llms-robotics-convergence',
            '04-module-four/02-voice-to-action-whisper',
            '04-module-four/03-cognitive-planning-llms-ros2',
            '04-module-four/04-capstone-autonomous-humanoid',
          ],
        },
      ],
    },
  ],

  // But you can create a sidebar manually
  /*
  tutorialSidebar: [
    'intro',
    'hello',
    {
      type: 'category',
      label: 'Tutorial',
      items: ['tutorial-basics/create-a-document'],
    },
  ],
   */
};

export default sidebars;