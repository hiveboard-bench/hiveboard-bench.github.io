import { defineConfig } from 'vitepress'

export default defineConfig({
  title: 'HiveBoard Documentation',
  description: 'Documentation for the HiveBoard manipulation benchmark.',
  lang: 'en-US',
  base: '/hivedocs/',
  outDir: '../dist/hivedocs',
  cleanUrls: true,
  lastUpdated: true,
  appearance: false,
  head: [
    ['meta', { name: 'theme-color', content: '#23527c' }],
    ['link', { rel: 'icon', href: '/hivedocs/images/hiveboard-mark.svg' }]
  ],
  themeConfig: {
    logo: '/images/hiveboard-mark.svg',
    siteTitle: 'HiveBoard',
    search: { provider: 'local' },
    nav: [
      { text: 'Contribute', link: '/contribute/' },
      { text: 'Project website', link: 'https://hiveboard-bench.github.io' },
      { text: 'GitHub', link: 'https://github.com/EESC-LabRoM/HiveBoard' }
    ],
    sidebar: [
      {
        text: 'Getting Started',
        items: [
          { text: 'Introduction', link: '/' },
          { text: 'Benchmark Overview', link: '/getting-started/overview' },
          { text: 'Getting Started', link: '/getting-started/quick-start' }
        ]
      },
      {
        text: 'Hardware',
        items: [
          { text: '3D Printing', link: '/hardware/printing' },
          { text: 'Assembly and Mounting', link: '/hardware/assembly' },
          { text: 'Module Reference', link: '/hardware/modules' }
        ]
      },
      {
        text: 'Benchmark',
        items: [
          { text: 'How to Perform Each Task', link: '/benchmark/tasks' },
          { text: 'Evaluation Runner', link: '/benchmark/evaluation-runner' },
          { text: 'Evaluation Protocol', link: '/benchmark/protocol' },
          { text: 'Trial Logging', link: '/benchmark/logging' },
          { text: 'Reporting Results', link: '/benchmark/results' }
        ]
      },
      {
        text: 'Simulation',
        items: [
          { text: 'Simulation Assets', link: '/simulation/assets' },
          { text: 'Isaac Lab Integration', link: '/simulation/isaac-lab' }
        ]
      },
      {
        text: 'Contributions',
        items: [
          { text: 'Open Calls', link: '/contribute/' },
          { text: 'Learning Datasets', link: '/contribute/evaluations' },
          { text: 'Activities of Daily Living', link: '/contribute/adl' },
          { text: 'Bimanual Manipulation', link: '/contribute/bimanual' },
          { text: 'Requirements and Credit', link: '/contribute/requirements' },
          { text: 'Adding an Attachment', link: '/guides/new-attachment' }
        ]
      },
      {
        text: 'Reference',
        items: [
          { text: 'Repository Structure', link: '/reference/repositories' },
          { text: 'Troubleshooting', link: '/reference/troubleshooting' },
          { text: 'Citation', link: '/reference/citation' }
        ]
      }
    ],
    socialLinks: [
      { icon: 'github', link: 'https://github.com/EESC-LabRoM/HiveBoard' }
    ],
    footer: {
      message: 'HiveBoard documentation',
      copyright: 'Copyright © 2026 HiveBoard contributors'
    },
    outline: { level: [2, 3], label: 'On this page' },
    docFooter: { prev: 'Previous', next: 'Next' },
    lastUpdated: { text: 'Updated' }
  }
})
