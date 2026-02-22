/**
 * Website Content Ingestion Module
 *
 * Robust ingestion for Docusaurus websites:
 * - Full sitemap coverage
 * - Browser-like fetching (avoids fallback pages)
 * - Duplicate content detection
 * - Clean text extraction
 */

const axios = require('axios');
const cheerio = require('cheerio');
const fs = require('fs');
const path = require('path');
const xml2js = require('xml2js');
const crypto = require('crypto');

class WebsiteIngestion {
  constructor() {
    this.baseUrl =
      'https://physical-ai-humanoid-robotics-book-seven-drab.vercel.app';
    this.timeout = 20000;

    // Used to prevent duplicate content ingestion
    this.seenContentHashes = new Set();
  }

  setBaseUrl(url) {
    this.baseUrl = url.endsWith('/') ? url.slice(0, -1) : url;
  }

  /**
   * Fetch HTML content using real browser headers
   * (prevents Docusaurus/Vercel fallback pages)
   */
  async fetchHtml(urlPath) {
    const fullUrl = urlPath.startsWith('http')
      ? urlPath
      : `${this.baseUrl}${urlPath}`;

    try {
      const response = await axios.get(fullUrl, {
        headers: {
          'User-Agent':
            'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 ' +
            '(KHTML, like Gecko) Chrome/121.0.0.0 Safari/537.36',
          Accept:
            'text/html,application/xhtml+xml,application/xml;q=0.9,image/webp,*/*;q=0.8',
          'Accept-Language': 'en-US,en;q=0.9',
          'Cache-Control': 'no-cache',
          Pragma: 'no-cache',
        },
        timeout: this.timeout,
        maxRedirects: 5,
        validateStatus: (s) => s >= 200 && s < 400,
      });

      return response.data;
    } catch (error) {
      console.error(`Failed to fetch ${fullUrl}: ${error.message}`);
      throw error;
    }
  }

  /**
   * Fetch sitemap XML safely
   */
  async fetchXml(url) {
    const response = await axios.get(url, {
      headers: {
        'User-Agent':
          'Mozilla/5.0 (Windows NT 10.0; Win64; x64)',
        Accept: 'application/xml,text/xml,*/*',
      },
      timeout: this.timeout,
    });

    if (response.status !== 200) {
      throw new Error(`HTTP ${response.status} from ${url}`);
    }

    return response.data;
  }

  /**
   * Extract only main Docusaurus markdown content
   */
  extractContent(html) {
    const $ = cheerio.load(html);

    $('nav, footer, .sidebar, .pagination-nav, .edit-this-page, .last-updated').remove();

    const mainHtml =
      $('.theme-doc-markdown, .markdown').first().html() || '';

    return this.cleanText($(mainHtml).text() || '');
  }

  cleanText(text) {
    return text
      .replace(/\s+/g, ' ')
      .replace(/[\n\r\t]/g, ' ')
      .trim();
  }

  extractMetadata(html, fullUrl) {
    const $ = cheerio.load(html);

    return {
      title: $('title').text() || 'Untitled',
      url: fullUrl,
      description: $('meta[name="description"]').attr('content') || '',
      language: $('html').attr('lang') || 'en',
      timestamp: new Date().toISOString(),
    };
  }

  /**
   * Ingest a single URL (with duplicate content protection)
   */
  async ingestUrl(urlPath) {
    const html = await this.fetchHtml(urlPath);
    const content = this.extractContent(html);

    if (!content || content.length < 200) {
      throw new Error('Empty or invalid page content');
    }

    // 🔐 Content deduplication (CRITICAL)
    const hash = crypto.createHash('sha256').update(content).digest('hex');

    if (this.seenContentHashes.has(hash)) {
      throw new Error('Duplicate page content detected');
    }

    this.seenContentHashes.add(hash);

    const fullUrl = `${this.baseUrl}${urlPath}`;
    const metadata = this.extractMetadata(html, fullUrl);

    return {
      url: urlPath,
      content,
      metadata,
      timestamp: new Date().toISOString(),
    };
  }

  /**
   * Ingest multiple URLs safely
   */
  async ingestMultipleUrls(urlPaths) {
    const results = [];

    for (const url of urlPaths) {
      try {
        const result = await this.ingestUrl(url);
        results.push(result);
      } catch (err) {
        console.warn(`Skipping ${url}: ${err.message}`);
      }
    }

    return results;
  }

  /**
   * Parse sitemap.xml with FULL coverage
   */
  async parseSitemap(sitemapSource) {
    let xml;

    if (sitemapSource.startsWith('http')) {
      console.log(`Fetching sitemap: ${sitemapSource}`);
      xml = await this.fetchXml(sitemapSource);
    } else {
      xml = fs.readFileSync(sitemapSource, 'utf8');
    }

    const parser = new xml2js.Parser();
    const parsed = await parser.parseStringPromise(xml);

    if (!parsed.urlset?.url) {
      throw new Error('Invalid sitemap.xml structure');
    }

    const paths = parsed.urlset.url
      .map((u) => u.loc?.[0])
      .filter(Boolean)
      .map((u) => {
        try {
          return new URL(u).pathname;
        } catch {
          return null;
        }
      })
      .filter(Boolean)
      .filter((p) => {
        if (!p.startsWith('/docs')) return false;
        if (p.includes('/tags') || p.includes('/authors')) return false;
        if (p.includes('/blog')) return false;
        return true;
      });

    return Array.from(new Set(paths));
  }

  saveContent(content, filePath) {
    fs.mkdirSync(path.dirname(filePath), { recursive: true });
    fs.writeFileSync(filePath, JSON.stringify(content, null, 2), 'utf8');
  }

  loadContent(filePath) {
    return JSON.parse(fs.readFileSync(filePath, 'utf8'));
  }
}

module.exports = WebsiteIngestion;
