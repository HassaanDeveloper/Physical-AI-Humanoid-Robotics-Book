import React, { useState, useRef, useEffect } from 'react';
import styles from './styles.module.css';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import clsx from 'clsx'; // Import clsx

const Chatbot = ({ isOpen }) => { // Accept isOpen prop
  const {siteConfig} = useDocusaurusContext();
  const {API_ENDPOINT} = siteConfig.customFields;

  const [messages, setMessages] = useState([
    { text: "Hello! I'm your book assistant. How can I help you today?", sender: 'bot', isWelcome: true }
  ]); // Add default welcome message
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);
  const messageContainerRef = useRef(null);

  // Auto-scroll to bottom when new messages are added
  useEffect(() => {
    if (messageContainerRef.current) {
      messageContainerRef.current.scrollTop = messageContainerRef.current.scrollHeight;
    }
  }, [messages, isLoading]);

  const handleSend = async () => {
    if (input.trim() && !isLoading) {
      const userMessage = { text: input, sender: 'user' };
      setMessages(prev => [...prev, userMessage]);
      setInput('');
      setIsLoading(true);
      setError(null);

      try {
        const response = await fetch(`${API_ENDPOINT}/chat`, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ query: input, max_chunks: 5 }),
        });

        if (!response.ok) {
          const errorData = await response.json().catch(() => ({}));
          throw new Error(errorData.detail || `HTTP error! status: ${response.status}`);
        }

        const data = await response.json();

        const botMessage = {
          text: data.response || 'Sorry, I could not process your request.',
          sender: 'bot',
          sources: data.retrieved_chunks?.slice(0, 2) || [],
        };
        setMessages(prev => [...prev, botMessage]);

      } catch (error) {
        console.error('Error sending message:', error);
        setError(error.message);
        const errorMessage = {
          text: error.message === 'Failed to fetch'
            ? 'Unable to connect to the backend. Please ensure it is running.'
            : `Error: ${error.message}`,
          sender: 'bot',
          isError: true
        };
        setMessages(prev => [...prev, errorMessage]);
      } finally {
        setIsLoading(false);
      }
    }
  };

  return (
    <div className={clsx(styles.chatbotContainer, isOpen && styles.chatbotContainerOpen)}> {/* Apply class based on isOpen */}
      <div className={styles.chatbotHeader}>
        AI Chat Assistant <br /> <span className={styles.headerSubtitle}>Ask me anything about Book content!</span>
      </div>
      <div className={styles.messageContainer} ref={messageContainerRef}>
        {messages.map((msg, index) => (
          <div key={index} className={clsx(styles.message, styles[msg.sender], msg.isWelcome && styles.welcomeMessage)}>
            <div className={styles.avatar}>
              {msg.sender === 'user' ? 'U' : 'AI'}
            </div>
            <div className={styles.messageContent}>
              <div>{msg.text}</div>
            </div>
          </div>
        ))}
        {isLoading && (
          <div className={`${styles.message} ${styles.bot}`}>
            <div className={styles.avatar}>AI</div>
            <div className={styles.messageContent}>
              Thinking...
            </div>
          </div>
        )}
        {error && <div className={styles.error}>{error}</div>}
      </div>
      <div className={styles.inputContainer}>
        <input
          type="text"
          value={input}
          onChange={(e) => setInput(e.target.value)}
          onKeyPress={(e) => e.key === 'Enter' && !isLoading && handleSend()}
          placeholder="Ask a question..."
          disabled={isLoading}
        />
        <button onClick={handleSend} disabled={isLoading}>
          <svg xmlns="http://www.w3.org/2000/svg" width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
            <line x1="22" y1="2" x2="11" y2="13"></line>
            <polygon points="22 2 15 22 11 13 2 9 22 2"></polygon>
          </svg>
        </button>
      </div>
    </div>
  );
};

export default Chatbot;
