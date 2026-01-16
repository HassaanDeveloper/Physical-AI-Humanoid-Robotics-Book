import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HeroSection from '@site/src/components/HeroSection';
import ModulesOverview from '@site/src/components/ModulesOverview';

import Heading from '@theme/Heading';
import styles from './index.module.css';

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title}`}
      // description="A comprehensive guide to Physical AI and Humanoid Robotics"
      >
      <HeroSection />
      <main>
        <ModulesOverview />
      </main>
    </Layout>
  );
}