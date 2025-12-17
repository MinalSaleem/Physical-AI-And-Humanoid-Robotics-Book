import React, { useState, useEffect, useRef } from 'react';
import axios from 'axios';
import { v4 as uuidv4 } from 'uuid';

const BACKEND_URL = 'https://book-chatbot-1.onrender.com/chat';  // Your live Render URL

const Chatbot = () => {
  const [open, setOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [input, setInput] = useState('');
  const [loading, setLoading] = useState(false);
  const [sessionId, setSessionId] = useState('');
  const [selectedText, setSelectedText] = useState('');
  const messagesEndRef = useRef(null);

  useEffect(() => {
    const stored = localStorage.getItem('chat_session_id');
    if (stored) setSessionId(stored);
  }, []);

  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  const handleSelectedText = () => {
    const selection = window.getSelection().toString().trim();
    if (selection) {
      setSelectedText(selection);
      setMessages((prev) => [...prev, { role: 'system', content: `Using selected text: "${selection.substring(0, 100)}..."` }]);
      alert('Selected text captured! Now ask your question about it.');
    } else {
      alert('Please select some text from the book first.');
    }
  };

  const sendMessage = async () => {
    if (!input.trim() || loading) return;

    const userMessage = { role: 'user', content: input };
    setMessages((prev) => [...prev, userMessage]);
    setLoading(true);
    setInput('');

    try {
      const res = await axios.post(BACKEND_URL, {
        query: input,
        selected_text: selectedText || null,
        session_id: sessionId || null,
      }, { timeout: 120000 });  // Timeout for Render cold starts

      const { answer, session_id } = res.data;
      setMessages((prev) => [...prev, { role: 'assistant', content: answer }]);
      setSessionId(session_id);
      localStorage.setItem('chat_session_id', session_id);
      setSelectedText('');  // Clear after send
    } catch (err) {
      console.error(err);
      setMessages((prev) => [...prev, { role: 'assistant', content: 'Sorry, the server is waking up or there was an error. Try again in a minute.' }]);
    } finally {
      setLoading(false);
    }
  };

  return (
    <>
      {/* Floating Bubble Button */}
      <button
        onClick={() => setOpen(!open)}
        style={{
          position: 'fixed',
          bottom: '20px',
          right: '20px',
          width: '60px',
          height: '60px',
          borderRadius: '50%',
          background: '#0070f3',
          color: 'white',
          border: 'none',
          fontSize: '24px',
          boxShadow: '0 4px 12px rgba(0,0,0,0.3)',
          cursor: 'pointer',
          zIndex: 1000,
        }}
      >
        💬
      </button>

      {/* Chat Window (Popup) */}
      {open && (
        <div style={{
          position: 'fixed',
          bottom: '90px',
          right: '20px',
          width: '380px',
          height: '600px',
          background: 'white',
          borderRadius: '12px',
          boxShadow: '0 8px 32px rgba(0,0,0,0.3)',
          display: 'flex',
          flexDirection: 'column',
          zIndex: 1000,
          overflow: 'hidden',
          maxWidth: '90vw',  // Mobile-friendly
        }}>
          <div style={{ background: '#0070f3', color: 'white', padding: '16px', fontWeight: 'bold', textAlign: 'center' }}>
            Book AI Tutor
          </div>

          <div style={{ flex: 1, padding: '16px', overflowY: 'auto', background: '#f9f9f9' }}>
            {messages.length === 0 && (
              <p style={{ color: '#666', fontStyle: 'italic', textAlign: 'center' }}>
                Ask about Physical AI & Humanoid Robotics!<br />
                Tip: Select text → "Use Selected Text" → ask.
              </p>
            )}
            {messages.map((msg, i) => (
              <div key={i} style={{
                marginBottom: '12px',
                textAlign: msg.role === 'user' ? 'right' : 'left',
              }}>
                <div style={{
                  display: 'inline-block',
                  maxWidth: '80%',
                  padding: '10px 14px',
                  borderRadius: '18px',
                  background: msg.role === 'user' ? '#0070f3' : '#e5e5ea',
                  color: msg.role === 'user' ? 'white' : 'black',
                }}>
                  {msg.content}
                </div>
              </div>
            ))}
            {loading && (
              <div style={{ textAlign: 'left' }}>
                <div style={{ background: '#e5e5ea', display: 'inline-block', padding: '10px 14px', borderRadius: '18px' }}>
                  Thinking...
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          <div style={{ padding: '12px', borderTop: '1px solid #eee', background: 'white' }}>
            <button
              onClick={handleSelectedText}
              style={{
                width: '100%',
                padding: '8px',
                marginBottom: '8px',
                background: '#f0f0f0',
                border: 'none',
                borderRadius: '8px',
                cursor: 'pointer',
              }}
            >
              📄 Use Selected Text
            </button>
            <div style={{ display: 'flex' }}>
              <input
                value={input}
                onChange={(e) => setInput(e.target.value)}
                onKeyPress={(e) => e.key === 'Enter' && sendMessage()}
                placeholder="Ask a question..."
                style={{ flex: 1, padding: '12px', border: '1px solid #ccc', borderRadius: '8px 0 0 8px' }}
              />
              <button
                onClick={sendMessage}
                disabled={loading}
                style={{
                  padding: '12px 16px',
                  background: '#0070f3',
                  color: 'white',
                  border: 'none',
                  borderRadius: '0 8px 8px 0',
                  cursor: 'pointer',
                }}
              >
                Send
              </button>
            </div>
          </div>
        </div>
      )}
    </>
  );
};

export default Chatbot;