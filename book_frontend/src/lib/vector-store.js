/**
 * Qdrant Vector Storage Module
 *
 * This module handles storing and retrieving vector embeddings in Qdrant,
 * supporting semantic search and RAG applications.
 */

const { QdrantClient } = require('@qdrant/js-client-rest');

class QdrantVectorStore {
  constructor() {
    this.client = null;
    this.collectionName = 'book_content';
    this.embeddingDimension = 1024; // Default for Cohere embed-english-v3.0
    this.batchSize = 100; // Qdrant batch size
  }

  /**
   * Initialize Qdrant client
   * @param {string} url - Qdrant server URL
   * @param {string} apiKey - Qdrant API key (optional for cloud)
   */
  async initialize(url, apiKey = null) {
    try {
      this.client = new QdrantClient({
        url: url,
        apiKey: apiKey
      });

      // Test connection
      await this.client.getCollections();
      console.log('Qdrant client initialized successfully');
      return true;
    } catch (error) {
      console.error('Failed to initialize Qdrant client:', error.message);
      throw error;
    }
  }

  /**
   * Set collection name
   * @param {string} name - Collection name
   */
  setCollectionName(name) {
    this.collectionName = name;
  }

  /**
   * Set embedding dimension
   * @param {number} dimension - Embedding dimension
   */
  setEmbeddingDimension(dimension) {
    this.embeddingDimension = dimension;
  }

  /**
   * Check if collection exists
   * @returns {Promise<boolean>} True if collection exists
   */
  async collectionExists() {
    try {
      const collections = await this.client.getCollections();
      return collections.collections.some(col => col.name === this.collectionName);
    } catch (error) {
      console.error('Failed to check collection existence:', error.message);
      return false;
    }
  }

  /**
   * Create a new collection
   * @returns {Promise<boolean>} True if collection created successfully
   */
  async createCollection() {
    try {
      await this.client.createCollection(this.collectionName, {
        vectors: {
          size: this.embeddingDimension,
          distance: 'Cosine'
        }
      });

      console.log(`Collection ${this.collectionName} created successfully`);
      return true;
    } catch (error) {
      console.error('Failed to create collection:', error.message);
      throw error;
    }
  }

  /**
   * Ensure collection exists (create if not)
   * @returns {Promise<boolean>} True if collection is ready
   */
  async ensureCollection() {
    const exists = await this.collectionExists();

    if (!exists) {
      return await this.createCollection();
    }

    return true;
  }

  /**
   * Prepare payload for Qdrant from chunk data
   * @param {Object} chunk - Chunk object with embedding
   * @returns {Object} Qdrant payload
   */
  preparePayload(chunk) {
    return {
      id: this.generateId(chunk),
      vector: chunk.embedding,
      payload: {
        text: chunk.text,
        url: chunk.metadata.url,
        title: chunk.metadata.title,
        chunk_index: chunk.metadata.chunk_index,
        chunk_total: chunk.metadata.chunk_total,
        timestamp: chunk.metadata.timestamp,
        source: 'book-hackathon',
        ...chunk.metadata
      }
    };
  }

  /**
   * Generate deterministic ID for chunk
   * @param {Object} chunk - Chunk object
   * @returns {string} Generated ID
   */
  generateId(chunk) {
    // Create deterministic UUID based on URL and chunk index
    const uniqueId = `${chunk.metadata.url}_${chunk.metadata.chunk_index}`;
    return this.generateUUID(uniqueId);
  }

  /**
   * Simple hash function for ID generation
   * @param {string} str - String to hash
   * @returns {string} Hash string
   */
  simpleHash(str) {
    let hash = 0;
    for (let i = 0; i < str.length; i++) {
      const char = str.charCodeAt(i);
      hash = ((hash << 5) - hash) + char;
      hash = hash & hash; // Convert to 32bit integer
    }
    return Math.abs(hash).toString(16);
  }

  /**
   * Generate deterministic UUID for chunk
   * @param {string} uniqueId - Unique identifier string
   * @returns {string} Generated UUID
   */
  generateUUID(uniqueId) {
    // Simple deterministic UUID-like string generation
    const hash = this.simpleHash(uniqueId);
    // Create a UUID-like format: 8-4-4-4-12 hex digits
    const paddedHash = hash.padStart(32, '0');
    return `${paddedHash.substring(0, 8)}-${paddedHash.substring(8, 12)}-${paddedHash.substring(12, 16)}-${paddedHash.substring(16, 20)}-${paddedHash.substring(20, 32)}`;
  }

  /**
   * Store chunks in Qdrant
   * @param {Array<Object>} chunks - Array of chunk objects with embeddings
   * @returns {Promise<Array<Object>>} Array of operation results
   */
  async storeChunks(chunks) {
    if (!this.client) {
      throw new Error('Qdrant client not initialized');
    }

    await this.ensureCollection();

    // ✅ FILTER INVALID EMBEDDINGS
    const validChunks = chunks.filter(chunk =>
      Array.isArray(chunk.embedding) &&
      chunk.embedding.length === this.embeddingDimension
    );

    if (validChunks.length === 0) {
      throw new Error('No valid embeddings to store. All embeddings are null or invalid.');
    }

    const points = validChunks.map(chunk => this.preparePayload(chunk));

    const results = [];

    console.log('Inserting points:', {
      total: points.length,
      vectorLength: points[0].vector.length,
      collection: this.collectionName
    });

    // Process in batches
    for (let i = 0; i < points.length; i += this.batchSize) {
      const batch = points.slice(i, i + this.batchSize);



      try {
        const response = await this.client.upsert(this.collectionName, {
          wait: true,
          points: batch
        });

        results.push(response);
        console.log(`Stored batch ${i}-${i + this.batchSize} successfully`);
      } catch (error) {
        console.error(`Failed to store batch ${i}-${i + this.batchSize}:`, error.message);
        throw error;
      }
    }
    const count = await this.countPoints();
    console.log('Current Qdrant point count:', count);

    return results;

  }


  /**
   * Search for similar vectors
   * @param {Array<number>} queryVector - Query embedding vector
   * @param {number} limit - Number of results to return
   * @param {Object} filter - Optional filter object
   * @returns {Promise<Array<Object>>} Array of search results
   */
  async search(queryVector, limit = 5, filter = null) {
    if (!this.client) {
      throw new Error('Qdrant client not initialized');
    }

    try {
      const response = await this.client.search(this.collectionName, {
        vector: queryVector,
        limit: limit,
        filter: filter,
        with_payload: true,
        with_vectors: false
      });

      return response;
    } catch (error) {
      console.error('Failed to search:', error.message);
      throw error;
    }
  }

  /**
   * Search with text query (generates embedding first)
   * @param {string} queryText - Query text
   * @param {number} limit - Number of results to return
   * @param {EmbeddingGenerator} embeddingGenerator - Embedding generator instance
   * @returns {Promise<Array<Object>>} Array of search results
   */
  async searchText(queryText, limit = 5, embeddingGenerator) {
    if (!embeddingGenerator) {
      throw new Error('Embedding generator required for text search');
    }

    const queryVector = await embeddingGenerator.generateQueryEmbedding(queryText);
    return await this.search(queryVector, limit);
  }

  /**
   * Get collection information
   * @returns {Promise<Object>} Collection information
   */
  async getCollectionInfo() {
    try {
      return await this.client.getCollection(this.collectionName);
    } catch (error) {
      console.error('Failed to get collection info:', error.message);
      throw error;
    }
  }

  /**
   * Delete collection
   * @returns {Promise<boolean>} True if collection deleted
   */
  async deleteCollection() {
    try {
      await this.client.deleteCollection(this.collectionName);
      console.log(`Collection ${this.collectionName} deleted successfully`);
      return true;
    } catch (error) {
      console.error('Failed to delete collection:', error.message);
      throw error;
    }
  }

  /**
   * Count points in collection
   * @returns {Promise<number>} Number of points in collection
   */
  async countPoints() {
    try {
      const info = await this.getCollectionInfo();
      return info.points_count || 0;
    } catch (error) {
      console.error('Failed to count points:', error.message);
      return 0;
    }
  }

  /**
   * Get point by ID
   * @param {string} pointId - Point ID
   * @returns {Promise<Object>} Point data
   */
  async getPoint(pointId) {
    try {
      return await this.client.getPoint(this.collectionName, pointId);
    } catch (error) {
      console.error(`Failed to get point ${pointId}:`, error.message);
      throw error;
    }
  }

  /**
   * Delete point by ID
   * @param {string} pointId - Point ID
   * @returns {Promise<boolean>} True if point deleted
   */
  async deletePoint(pointId) {
    try {
      await this.client.deletePoint(this.collectionName, pointId);
      return true;
    } catch (error) {
      console.error(`Failed to delete point ${pointId}:`, error.message);
      throw error;
    }
  }

  /**
   * Update point
   * @param {string} pointId - Point ID
   * @param {Object} payload - New payload data
   * @returns {Promise<boolean>} True if point updated
   */
  async updatePoint(pointId, payload) {
    try {
      await this.client.updatePoint(this.collectionName, pointId, payload);
      return true;
    } catch (error) {
      console.error(`Failed to update point ${pointId}:`, error.message);
      throw error;
    }
  }

  /**
   * Scroll through all points (for debugging/export)
   * @param {number} limit - Batch size
   * @returns {Promise<Array<Object>>} Array of all points
   */
  async scrollAllPoints(limit = 100) {
    const allPoints = [];
    let offset = null;

    try {
      while (true) {
        const response = await this.client.scroll(this.collectionName, {
          limit: limit,
          offset: offset,
          with_payload: true,
          with_vectors: false
        });

        if (!response.points || response.points.length === 0) {
          break;
        }

        allPoints.push(...response.points);
        offset = response.next_offset;

        if (!offset) {
          break;
        }
      }

      return allPoints;
    } catch (error) {
      console.error('Failed to scroll points:', error.message);
      throw error;
    }
  }
}

module.exports = QdrantVectorStore;