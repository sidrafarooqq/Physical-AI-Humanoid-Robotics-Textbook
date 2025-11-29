import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/ur/docs',
    component: ComponentCreator('/ur/docs', '857'),
    routes: [
      {
        path: '/ur/docs',
        component: ComponentCreator('/ur/docs', 'af4'),
        routes: [
          {
            path: '/ur/docs',
            component: ComponentCreator('/ur/docs', 'c95'),
            routes: [
              {
                path: '/ur/docs/Module-1-The Robotic-Nervous-System/chapter1',
                component: ComponentCreator('/ur/docs/Module-1-The Robotic-Nervous-System/chapter1', 'f40'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ur/docs/Module-1-The Robotic-Nervous-System/chapter2',
                component: ComponentCreator('/ur/docs/Module-1-The Robotic-Nervous-System/chapter2', '6d3'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ur/docs/Module-1-The Robotic-Nervous-System/chapter3',
                component: ComponentCreator('/ur/docs/Module-1-The Robotic-Nervous-System/chapter3', '621'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ur/docs/Module-1-The Robotic-Nervous-System/chapter4',
                component: ComponentCreator('/ur/docs/Module-1-The Robotic-Nervous-System/chapter4', 'f63'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ur/docs/Module-2-The-Digital-Twin-Gazebo-&-Unity/chapter1',
                component: ComponentCreator('/ur/docs/Module-2-The-Digital-Twin-Gazebo-&-Unity/chapter1', '7d5'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ur/docs/Module-2-The-Digital-Twin-Gazebo-&-Unity/chapter2',
                component: ComponentCreator('/ur/docs/Module-2-The-Digital-Twin-Gazebo-&-Unity/chapter2', 'f85'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ur/docs/Module-2-The-Digital-Twin-Gazebo-&-Unity/chapter3',
                component: ComponentCreator('/ur/docs/Module-2-The-Digital-Twin-Gazebo-&-Unity/chapter3', '226'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ur/docs/Module-2-The-Digital-Twin-Gazebo-&-Unity/chapter4',
                component: ComponentCreator('/ur/docs/Module-2-The-Digital-Twin-Gazebo-&-Unity/chapter4', '77b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ur/docs/Module-3-The-AI-Robot-Brain-NVIDIA-Isaac™/chapter1',
                component: ComponentCreator('/ur/docs/Module-3-The-AI-Robot-Brain-NVIDIA-Isaac™/chapter1', '5b0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ur/docs/Module-3-The-AI-Robot-Brain-NVIDIA-Isaac™/chapter2',
                component: ComponentCreator('/ur/docs/Module-3-The-AI-Robot-Brain-NVIDIA-Isaac™/chapter2', '197'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ur/docs/Module-3-The-AI-Robot-Brain-NVIDIA-Isaac™/chapter3',
                component: ComponentCreator('/ur/docs/Module-3-The-AI-Robot-Brain-NVIDIA-Isaac™/chapter3', 'c82'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ur/docs/Module-3-The-AI-Robot-Brain-NVIDIA-Isaac™/chapter4',
                component: ComponentCreator('/ur/docs/Module-3-The-AI-Robot-Brain-NVIDIA-Isaac™/chapter4', '9f5'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ur/docs/Module-4-Vision-Language-Action-VLA/chapter1',
                component: ComponentCreator('/ur/docs/Module-4-Vision-Language-Action-VLA/chapter1', '43b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ur/docs/Module-4-Vision-Language-Action-VLA/chapter2',
                component: ComponentCreator('/ur/docs/Module-4-Vision-Language-Action-VLA/chapter2', '73c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ur/docs/Module-4-Vision-Language-Action-VLA/chapter3',
                component: ComponentCreator('/ur/docs/Module-4-Vision-Language-Action-VLA/chapter3', '6fe'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ur/docs/Module-4-Vision-Language-Action-VLA/chapter4',
                component: ComponentCreator('/ur/docs/Module-4-Vision-Language-Action-VLA/chapter4', 'e64'),
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
    path: '/ur/',
    component: ComponentCreator('/ur/', '3b1'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
