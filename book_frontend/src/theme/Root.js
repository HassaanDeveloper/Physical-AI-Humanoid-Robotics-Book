import React, { useState, useCallback } from 'react';
import Chatbot from '@site/src/components/Chatbot';

const Root = ({children}) => {
  const [isOpen, setIsOpen] = useState(false);

  const toggleChatbot = useCallback(() => {
    setIsOpen(prev => !prev);
  }, []);

  return (
    <>
      {children}
      {isOpen && <Chatbot isOpen={isOpen} />}
      <button
        onClick={toggleChatbot}
        style={{
          position: 'fixed',
          bottom: '20px',
          right: '20px',
          zIndex: 1000,
          padding: '10px 20px',
          borderRadius: '50px',
          backgroundColor: '#007bff',
          color: 'white',
          border: 'none',
          cursor: 'pointer',
          boxShadow: '0 4px 8px rgba(0, 0, 0, 0.2)',
          fontSize: '16px',
          fontWeight: 'bold',
        }}
      >
        {isOpen ? 'Close Chat' : 'Open Chat'}
      </button>
    </>
  );
};

export default Root;