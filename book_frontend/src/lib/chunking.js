/**
 * Content Chunking and Preprocessing Module
 *
 * This module handles splitting content into appropriate chunks for embedding,
 * ensuring deterministic chunking and proper metadata preservation.
 */

class ContentChunking {
  constructor() {
    this.defaultChunkSize = 500; // tokens
    this.defaultOverlap = 50; // tokens
    this.maxChunkSize = 1000; // tokens
  }

  /**
   * Set chunking parameters
   * @param {number} chunkSize - Size of each chunk in tokens
   * @param {number} overlap - Overlap between chunks in tokens
   */
  setChunkingParameters(chunkSize, overlap) {
    this.defaultChunkSize = Math.min(chunkSize, this.maxChunkSize);
    this.defaultOverlap = Math.min(overlap, this.defaultChunkSize / 2);
  }

  /**
   * Estimate token count for text
   * @param {string} text - Text to estimate tokens for
   * @returns {number} Estimated token count
   */
  estimateTokenCount(text) {
    // Simple approximation: 4 characters per token (English average)
    return Math.ceil(text.length / 4);
  }

  /**
   * Split text into sentences
   * @param {string} text - Text to split
   * @returns {Array<string>} Array of sentences
   */
  splitIntoSentences(text) {
    // Split on common sentence terminators, keeping the terminator
    return text
      .split(/(?<=[.!?])\s+|\n\n+/)
      .filter(sentence => sentence && sentence.trim().length > 0)
      .map(sentence => sentence.trim());
  }

  /**
   * Create chunks from text with metadata preservation
   * @param {string} text - Text content to chunk
   * @param {Object} metadata - Original metadata
   * @param {number} [chunkSize] - Optional chunk size override
   * @param {number} [overlap] - Optional overlap override
   * @returns {Array<Object>} Array of chunk objects with metadata
   */
  createChunks(text, metadata, chunkSize = null, overlap = null) {
    const actualChunkSize = chunkSize || this.defaultChunkSize;
    const actualOverlap = overlap || this.defaultOverlap;

    const sentences = this.splitIntoSentences(text);
    const chunks = [];
    let currentChunk = '';
    let currentChunkTokens = 0;

    for (let i = 0; i < sentences.length; i++) {
      const sentence = sentences[i];
      const sentenceTokens = this.estimateTokenCount(sentence);

      // Check if adding this sentence would exceed chunk size
      if (currentChunkTokens + sentenceTokens > actualChunkSize && currentChunk.length > 0) {
        // Save current chunk
        chunks.push(this.createChunkObject(currentChunk, metadata, chunks.length));

        // Start new chunk with overlap
        const overlapStart = Math.max(0, i - Math.ceil(actualOverlap / 2));
        currentChunk = sentences.slice(overlapStart, i).join(' ') + ' ' + sentence;
        currentChunkTokens = this.estimateTokenCount(currentChunk);
      } else {
        // Add to current chunk
        currentChunk += (currentChunk.length > 0 ? ' ' : '') + sentence;
        currentChunkTokens += sentenceTokens;
      }
    }

    // Add final chunk if it exists
    if (currentChunk.length > 0) {
      chunks.push(this.createChunkObject(currentChunk, metadata, chunks.length));
    }

    return chunks;
  }

  /**
   * Create a chunk object with proper metadata
   * @param {string} text - Chunk text
   * @param {Object} originalMetadata - Original metadata
   * @param {number} chunkIndex - Index of this chunk
   * @returns {Object} Chunk object
   */
  createChunkObject(text, originalMetadata, chunkIndex) {
    return {
      text: text.trim(),
      metadata: {
        ...originalMetadata,
        chunk_index: chunkIndex,
        chunk_total: -1, // Will be updated later
        token_count: this.estimateTokenCount(text),
        timestamp: new Date().toISOString()
      }
    };
  }

  /**
   * Update chunk metadata with total count
   * @param {Array<Object>} chunks - Array of chunk objects
   */
  updateChunkMetadata(chunks) {
    chunks.forEach((chunk, index) => {
      chunk.metadata.chunk_total = chunks.length;
      chunk.metadata.chunk_index = index;
    });
  }

  /**
   * Process content through the complete chunking pipeline
   * @param {Object|Array<Object>} content - Content to process (single or multiple)
   * @returns {Array<Object>} Array of processed chunks
   */
  processContent(content) {
    const contents = Array.isArray(content) ? content : [content];
    let allChunks = [];

    for (const item of contents) {
      const chunks = this.createChunks(item.content, item.metadata);
      this.updateChunkMetadata(chunks);
      allChunks = allChunks.concat(chunks);
    }

    return allChunks;
  }

  /**
   * Preprocess text for better embedding quality
   * @param {string} text - Text to preprocess
   * @returns {string} Preprocessed text
   */
  preprocessText(text) {
    return text
      .replace(/\s+/g, ' ') // Normalize whitespace
      .replace(/[\n\r\t]/g, ' ') // Remove line breaks
      .replace(/\s*[.,;:!?]\s*/g, '$1 ') // Clean punctuation spacing
      .replace(/\s+'\s*/g, "' ") // Clean apostrophes
      .replace(/\s+"/g, ' "') // Clean quotes
      .replace(/"\s+/g, '" ') // Clean quotes
      .replace(/\s+-\s+/g, ' - ') // Clean hyphens
      .replace(/\s+\/\s+/g, ' / ') // Clean slashes
      .replace(/\s+\\\s+/g, ' \\ ') // Clean backslashes
      .replace(/\s+\|\s+/g, ' | ') // Clean pipes
      .replace(/\s+\*\s+/g, ' * ') // Clean asterisks
      .replace(/\s+\_\s+/g, ' _ ') // Clean underscores
      .replace(/\s+\`\s+/g, ' ` ') // Clean backticks
      .replace(/\s+\#\s+/g, ' # ') // Clean hashes
      .replace(/\s+\$\s+/g, ' $ ') // Clean dollar signs
      .replace(/\s+\%\s+/g, ' % ') // Clean percent signs
      .replace(/\s+\&\s+/g, ' & ') // Clean ampersands
      .replace(/\s+\+\s+/g, ' + ') // Clean plus signs
      .replace(/\s+\=\s+/g, ' = ') // Clean equals signs
      .replace(/\s+\@\s+/g, ' @ ') // Clean at signs
      .replace(/\s+\^\s+/g, ' ^ ') // Clean carets
      .replace(/\s+\~\s+/g, ' ~ ') // Clean tildes
      .replace(/\s+\`\s+/g, ' ` ') // Clean backticks
      .replace(/\s+\{\s+/g, ' { ') // Clean braces
      .replace(/\s+\}\s+/g, ' } ') // Clean braces
      .replace(/\s+\[\s+/g, ' [ ') // Clean brackets
      .replace(/\s+\]\s+/g, ' ] ') // Clean brackets
      .replace(/\s+\(\s+/g, ' ( ') // Clean parentheses
      .replace(/\s+\)\s+/g, ' ) ') // Clean parentheses
      .replace(/\s+\<\s+/g, ' < ') // Clean angle brackets
      .replace(/\s+\>\s+/g, ' > ') // Clean angle brackets
      .replace(/\s+\|\s+/g, ' | ') // Clean pipes
      .replace(/\s+\\\s+/g, ' \\ ') // Clean backslashes
      .replace(/\s+\/\s+/g, ' / ') // Clean slashes
      .replace(/\s+\:\s+/g, ' : ') // Clean colons
      .replace(/\s+\;\s+/g, ' ; ') // Clean semicolons
      .replace(/\s+\,\s+/g, ' , ') // Clean commas
      .replace(/\s+\.\s+/g, ' . ') // Clean periods
      .replace(/\s+\!\s+/g, ' ! ') // Clean exclamation marks
      .replace(/\s+\?\s+/g, ' ? ') // Clean question marks
      .replace(/\s+\'\s+/g, " ' ") // Clean single quotes
      .replace(/\s+\"\s+/g, ' " ') // Clean double quotes
      .replace(/\s+\`\s+/g, ' ` ') // Clean backticks
      .replace(/\s+\~\s+/g, ' ~ ') // Clean tildes
      .replace(/\s+\`\s+/g, ' ` ') // Clean backticks
      .replace(/\s+\*\s+/g, ' * ') // Clean asterisks
      .replace(/\s+\_\s+/g, ' _ ') // Clean underscores
      .replace(/\s+\#\s+/g, ' # ') // Clean hashes
      .replace(/\s+\$\s+/g, ' $ ') // Clean dollar signs
      .replace(/\s+\%\s+/g, ' % ') // Clean percent signs
      .replace(/\s+\&\s+/g, ' & ') // Clean ampersands
      .replace(/\s+\+\s+/g, ' + ') // Clean plus signs
      .replace(/\s+\=\s+/g, ' = ') // Clean equals signs
      .replace(/\s+\@\s+/g, ' @ ') // Clean at signs
      .replace(/\s+\^\s+/g, ' ^ ') // Clean carets
      .replace(/\s+\~\s+/g, ' ~ ') // Clean tildes
      .replace(/\s+\`\s+/g, ' ` ') // Clean backticks
      .replace(/\s+\{\s+/g, ' { ') // Clean braces
      .replace(/\s+\}\s+/g, ' } ') // Clean braces
      .replace(/\s+\[\s+/g, ' [ ') // Clean brackets
      .replace(/\s+\]\s+/g, ' ] ') // Clean brackets
      .replace(/\s+\(\s+/g, ' ( ') // Clean parentheses
      .replace(/\s+\)\s+/g, ' ) ') // Clean parentheses
      .replace(/\s+\<\s+/g, ' < ') // Clean angle brackets
      .replace(/\s+\>\s+/g, ' > ') // Clean angle brackets
      .replace(/\s+\|\s+/g, ' | ') // Clean pipes
      .replace(/\s+\\\s+/g, ' \\ ') // Clean backslashes
      .replace(/\s+\/\s+/g, ' / ') // Clean slashes
      .replace(/\s+\:\s+/g, ' : ') // Clean colons
      .replace(/\s+\;\s+/g, ' ; ') // Clean semicolons
      .replace(/\s+\,\s+/g, ' , ') // Clean commas
      .replace(/\s+\.\s+/g, ' . ') // Clean periods
      .replace(/\s+\!\s+/g, ' ! ') // Clean exclamation marks
      .replace(/\s+\?\s+/g, ' ? ') // Clean question marks
      .replace(/\s+\'\s+/g, " ' ") // Clean single quotes
      .replace(/\s+\"\s+/g, ' " ') // Clean double quotes
      .replace(/\s+\`\s+/g, ' ` ') // Clean backticks
      .replace(/\s+\~\s+/g, ' ~ ') // Clean tildes
      .trim();
  }
}

module.exports = ContentChunking;