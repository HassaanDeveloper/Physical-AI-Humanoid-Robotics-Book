/**
 * Configuration for the Ingestion Pipeline
 */

module.exports = {
  // Cohere API Configuration
  cohere: {
    apiKey: process.env.COHERE_API_KEY || '', // Set in environment variables
    model: 'embed-english-v3.0'
  },

  // Qdrant Configuration
  qdrant: {
    url: process.env.QDRANT_URL || 'http://localhost:6333',
    apiKey: process.env.QDRANT_API_KEY || null // Optional for cloud
  },

  // Website Ingestion Configuration
  ingestion: {
    baseUrl: process.env.BASE_URL || 'https://physical-ai-humanoid-robotics-book-seven-drab.vercel.app',
    userAgent: 'Book-Hackathon-Ingestion-Bot/1.0',
    timeout: 10000
  },

  // Chunking Configuration
  chunking: {
    chunkSize: 500, // tokens
    overlap: 50 // tokens
  },

  // Use sitemap for comprehensive ingestion instead of hardcoded URLs
  // Set to null to use sitemap-based ingestion, or provide specific URLs to override
  defaultUrls: null,

  // Sitemap configuration for comprehensive ingestion
  sitemapUrl: 'https://physical-ai-humanoid-robotics-book-seven-drab.vercel.app/sitemap.xml'
};

// Make configuration available to client-side code
if (typeof window !== 'undefined') {
  window.config = module.exports;
}