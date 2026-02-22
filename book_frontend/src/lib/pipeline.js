/**
 * Main Ingestion Pipeline
 *
 * This module orchestrates the complete website ingestion, embedding generation,
 * and vector storage pipeline for the Book Hackathon project.
 */

const WebsiteIngestion = require('./ingestion');
const ContentChunking = require('./chunking');
const EmbeddingGenerator = require('./embeddings');
const QdrantVectorStore = require('./vector-store');

class IngestionPipeline {
  constructor() {
    this.ingestion = new WebsiteIngestion();
    this.chunking = new ContentChunking();
    this.embeddings = new EmbeddingGenerator();
    this.vectorStore = new QdrantVectorStore();
  }

  /**
   * Initialize all components
   * @param {Object} config - Configuration object
   * @param {string} config.cohereApiKey - Cohere API key
   * @param {string} config.qdrantUrl - Qdrant server URL
   * @param {string} config.qdrantApiKey - Qdrant API key (optional)
   * @param {string} config.baseUrl - Base URL for website ingestion
   */
  async initialize(config) {
    // Initialize Cohere
    this.embeddings.initialize(config.cohereApiKey);

    // Initialize Qdrant
    await this.vectorStore.initialize(config.qdrantUrl, config.qdrantApiKey);

    // Set base URL for ingestion
    if (config.baseUrl) {
      this.ingestion.setBaseUrl(config.baseUrl);
    }

    // Set embedding dimension in vector store
    this.vectorStore.setEmbeddingDimension(this.embeddings.getEmbeddingDimension());

    console.log('Pipeline initialized successfully');
  }

  /**
   * Run complete ingestion pipeline
   * @param {Array<string>} urls - Array of URLs to ingest
   * @returns {Promise<Object>} Pipeline results
   */
  async runPipeline(urls) {
    const results = {
      ingestion: [],
      chunking: [],
      embeddings: [],
      storage: []
    };

    try {
      // Step 1: Website Ingestion
      console.log('Starting website ingestion...');
      results.ingestion = await this.ingestion.ingestMultipleUrls(urls);
      console.log(`Ingested ${results.ingestion.length} pages successfully`);

      // Step 2: Content Chunking
      console.log('Starting content chunking...');
      results.chunking = this.chunking.processContent(results.ingestion);
      console.log(`Created ${results.chunking.length} chunks`);

      // Step 3: Embedding Generation
      console.log('Starting embedding generation...');
      results.embeddings = await this.embeddings.generateChunkEmbeddings(results.chunking);
      console.log(`Generated embeddings for ${results.embeddings.length} chunks`);

      // Step 4: Vector Storage
      console.log('Starting vector storage...');
      results.storage = await this.vectorStore.storeChunks(results.embeddings);
      console.log(`Stored ${results.embeddings.length} chunks in Qdrant`);

      return results;

    } catch (error) {
      console.error('Pipeline failed:', error.message);
      throw error;
    }
  }

  /**
   * Search the vector store
   * @param {string} query - Search query
   * @param {number} limit - Number of results
   * @returns {Promise<Array<Object>>} Search results
   */
  async search(query, limit = 5) {
    return await this.vectorStore.searchText(query, limit, this.embeddings);
  }

  /**
   * Run pipeline using sitemap.xml for comprehensive ingestion
   * @param {string} sitemapPath - Path to sitemap.xml file
   * @returns {Promise<Object>} Pipeline results
   */
  async runPipelineFromSitemap(sitemapPath) {
    const urls = await this.ingestion.parseSitemap(sitemapPath);
    console.log(`Found ${urls.length} documentation URLs in sitemap`);
    return this.runPipeline(urls);
  }

  /**
   * Get pipeline statistics
   * @returns {Promise<Object>} Pipeline statistics
   */
  async getStatistics() {
    const pointsCount = await this.vectorStore.countPoints();
    const collectionInfo = await this.vectorStore.getCollectionInfo();

    return {
      points_count: pointsCount,
      collection_info: collectionInfo,
      embedding_dimension: this.embeddings.getEmbeddingDimension(),
      chunking_config: {
        chunk_size: this.chunking.defaultChunkSize,
        overlap: this.chunking.defaultOverlap
      }
    };
  }

  /**
   * Ingest from local markdown files (alternative to web ingestion)
   * @param {Array<string>} filePaths - Array of file paths
   * @returns {Promise<Array<Object>>} Ingested content
   */
  async ingestLocalFiles(filePaths) {
    const fs = require('fs');
    const path = require('path');
    const marked = require('marked'); // Would need to install

    const results = [];

    for (const filePath of filePaths) {
      try {
        const content = fs.readFileSync(filePath, 'utf8');
        const html = marked.parse(content); // Convert markdown to HTML

        // Create mock metadata
        const metadata = {
          title: path.basename(filePath, '.md'),
          url: filePath,
          source: 'local-file',
          timestamp: new Date().toISOString()
        };

        results.push({
          content: this.ingestion.extractContent(html),
          metadata
        });

      } catch (error) {
        console.error(`Failed to process ${filePath}:`, error.message);
        continue;
      }
    }

    return results;
  }

  /**
   * Export vector store data
   * @returns {Promise<Object>} Exported data
   */
  async exportData() {
    const points = await this.vectorStore.scrollAllPoints();
    const stats = await this.getStatistics();

    return {
      points,
      statistics: stats,
      timestamp: new Date().toISOString()
    };
  }

  /**
   * Clear vector store
   * @returns {Promise<boolean>} True if cleared successfully
   */
  async clearStore() {
    try {
      await this.vectorStore.deleteCollection();
      await this.vectorStore.createCollection();
      return true;
    } catch (error) {
      console.error('Failed to clear store:', error.message);
      return false;
    }
  }
}

module.exports = IngestionPipeline;