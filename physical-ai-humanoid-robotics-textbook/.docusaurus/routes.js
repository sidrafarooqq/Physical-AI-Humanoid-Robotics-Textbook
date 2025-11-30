import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/__docusaurus/debug',
    component: ComponentCreator('/__docusaurus/debug', '5ff'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/config',
    component: ComponentCreator('/__docusaurus/debug/config', '5ba'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/content',
    component: ComponentCreator('/__docusaurus/debug/content', 'a2b'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/globalData',
    component: ComponentCreator('/__docusaurus/debug/globalData', 'c3c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/metadata',
    component: ComponentCreator('/__docusaurus/debug/metadata', '156'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/registry',
    component: ComponentCreator('/__docusaurus/debug/registry', '88c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/routes',
    component: ComponentCreator('/__docusaurus/debug/routes', '000'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', 'a3f'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', '9bf'),
        routes: [
          {
            path: '/docs',
            component: ComponentCreator('/docs', '3a1'),
            routes: [
              {
                path: '/docs/Module-1-The Robotic-Nervous-System/chapter1',
                component: ComponentCreator('/docs/Module-1-The Robotic-Nervous-System/chapter1', '952'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/Module-1-The Robotic-Nervous-System/chapter2',
                component: ComponentCreator('/docs/Module-1-The Robotic-Nervous-System/chapter2', 'fe8'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/Module-1-The Robotic-Nervous-System/chapter3',
                component: ComponentCreator('/docs/Module-1-The Robotic-Nervous-System/chapter3', 'dc1'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/Module-1-The Robotic-Nervous-System/chapter4',
                component: ComponentCreator('/docs/Module-1-The Robotic-Nervous-System/chapter4', 'cbd'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/Module-2-The-Digital-Twin-Gazebo-&-Unity/chapter1',
                component: ComponentCreator('/docs/Module-2-The-Digital-Twin-Gazebo-&-Unity/chapter1', '01e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/Module-2-The-Digital-Twin-Gazebo-&-Unity/chapter2',
                component: ComponentCreator('/docs/Module-2-The-Digital-Twin-Gazebo-&-Unity/chapter2', '490'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/Module-2-The-Digital-Twin-Gazebo-&-Unity/chapter3',
                component: ComponentCreator('/docs/Module-2-The-Digital-Twin-Gazebo-&-Unity/chapter3', '99c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/Module-2-The-Digital-Twin-Gazebo-&-Unity/chapter4',
                component: ComponentCreator('/docs/Module-2-The-Digital-Twin-Gazebo-&-Unity/chapter4', '808'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/Module-3-The-AI-Robot-Brain-NVIDIA-Isaac™/chapter1',
                component: ComponentCreator('/docs/Module-3-The-AI-Robot-Brain-NVIDIA-Isaac™/chapter1', 'f8c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/Module-3-The-AI-Robot-Brain-NVIDIA-Isaac™/chapter2',
                component: ComponentCreator('/docs/Module-3-The-AI-Robot-Brain-NVIDIA-Isaac™/chapter2', '081'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/Module-3-The-AI-Robot-Brain-NVIDIA-Isaac™/chapter3',
                component: ComponentCreator('/docs/Module-3-The-AI-Robot-Brain-NVIDIA-Isaac™/chapter3', 'fb0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/Module-3-The-AI-Robot-Brain-NVIDIA-Isaac™/chapter4',
                component: ComponentCreator('/docs/Module-3-The-AI-Robot-Brain-NVIDIA-Isaac™/chapter4', 'f31'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/Module-4-Vision-Language-Action-VLA/chapter1',
                component: ComponentCreator('/docs/Module-4-Vision-Language-Action-VLA/chapter1', '21d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/Module-4-Vision-Language-Action-VLA/chapter2',
                component: ComponentCreator('/docs/Module-4-Vision-Language-Action-VLA/chapter2', '8d5'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/Module-4-Vision-Language-Action-VLA/chapter3',
                component: ComponentCreator('/docs/Module-4-Vision-Language-Action-VLA/chapter3', 'eb0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/Module-4-Vision-Language-Action-VLA/chapter4',
                component: ComponentCreator('/docs/Module-4-Vision-Language-Action-VLA/chapter4', '3b6'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/',
    component: ComponentCreator('/', '2e1'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
