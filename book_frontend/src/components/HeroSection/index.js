import React from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './styles.module.css';



export default function HeroSection() {
  const { siteConfig } = useDocusaurusContext();

  return (
    <section className={styles.hero}>
      <div className={styles.heroBackground}>
        <img src="/img/book_background.png" alt="Hero Background" />
      </div>
      <div className={styles.heroOverlay}>
        <div className={styles.heroContainer}>
          <div className={styles.heroContent}>
            <h1 className={styles.heroTitle}>{siteConfig.title}</h1>
            <p className={styles.heroTagline}>{siteConfig.tagline}</p>
            <p className={styles.heroDescription}>
              A comprehensive guide to embodied intelligence and humanoid robots
            </p>
            <Link
              className={styles.ctaButton}
              to="/docs/intro">
              Start Reading
            </Link>
          </div>
          <div className={styles.heroImage}>
            <img src="/img/book_heroImage.png" alt="Hero Image" className={styles.heroIllustration} />
          </div>
        </div>
      </div>
    </section>
  );
}