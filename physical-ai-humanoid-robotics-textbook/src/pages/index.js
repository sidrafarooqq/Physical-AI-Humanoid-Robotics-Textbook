import React, { useState, useEffect } from "react";
import "./../css/custom.css";

export default function HomePage() {
  const [mobileOpen, setMobileOpen] = useState(false);
  const [scrolled, setScrolled] = useState(false);

  useEffect(() => {
    function onScroll() {
      setScrolled(window.scrollY > 20);
    }
    onScroll();
    window.addEventListener("scroll", onScroll);
    return () => window.removeEventListener("scroll", onScroll);
  }, []);

  return (
    <div className="docubook-app">
      {/* NAVBAR */}
      <nav className={`navbar ${scrolled ? "nav-scrolled" : ""}`}>
        <div className="navbar-wrapper">
          <div className="nav-left">
            <a className="brand" href="/">
              <span className="brand-icon">📘</span>
              <span className="brand-text">DocuBook</span>
            </a>
          </div>

          <div className={`nav-center ${mobileOpen ? "open" : ""}`}>
            <ul className="nav-menu" role="menu">
              <li role="none"><a role="menuitem" href="/">Home</a></li>
              <li role="none"><a role="menuitem" href="/">Chapters</a></li>
              <li role="none"><a role="menuitem" href="/">Bonus</a></li>
              <li role="none"><a role="menuitem" href="/">Urdu</a></li>
            </ul>
          </div>

          <div className="nav-right">
            <a className="cta" href="/docs/Module-1-The%20Robotic-Nervous-System/chapter1">Start Reading</a>


            {/* Mobile Hamburger */}
            <button
              className={`hamburger ${mobileOpen ? "is-open" : ""}`}
              aria-label="Toggle menu"
              aria-expanded={mobileOpen}
              onClick={() => setMobileOpen(!mobileOpen)}
            >
              <span />
              <span />
              <span />
            </button>
          </div>
        </div>
      </nav>

      {/* HERO */}
      <header className="hero">
        <div className="hero-inner">
          <div className="hero-badge">New • Updated 2025</div>
          <h1 className="hero-title">Build Modern Apps — Step by Step</h1>
          <p className="hero-sub">
            A complete DocuSource book with examples, Urdu translations and practical bonuses.
            Learn from scratch and ship production-ready features.
          </p>

          <div className="hero-actions">
            <a className="btn primary" href="/docs/Module-1-The%20Robotic-Nervous-System/chapter1">Start Reading</a>
            <a className="btn outline" href="/">Read in Urdu</a>
          </div>

          <div className="hero-visual" aria-hidden>
            <div className="cover-card">
              <div className="cover-title">DocuBook</div>
              <div className="cover-sub">Practical guide • 2025</div>
            </div>
            <div className="floating-cards">
              <div className="mini-card">Auth</div>
              <div className="mini-card">Personalization</div>
              <div className="mini-card">Examples</div>
            </div>
          </div>
        </div>
      </header>

      {/* EXPLORE GRID */}
      <main className="main-section">
        <section className="section intro">
          <p className="section-desc">Jump to chapters, read bonuses, or open the Urdu translation.</p>
        </section>

        <section className="grid-section">
          <a className="card fade-up" href="/">
            <div className="card-head">Introduction</div>
            <p>Start with the basics — structure, tooling and simple examples.</p>
          </a>

          <a className="card fade-up" href="/" style={{ animationDelay: "80ms" }}>
            <div className="card-head">Core Concepts</div>
            <p>State, props, routing, and component patterns explained clearly.</p>
          </a>

          <a className="card fade-up" href="/" style={{ animationDelay: "160ms" }}>
            <div className="card-head">Advanced Techniques</div>
            <p>Performance, testing, middleware, and deployment workflows.</p>
          </a>

          <a className="card fade-up" href="/" style={{ animationDelay: "240ms" }}>
            <div className="card-head">Authentication</div>
            <p>Secure flows, tokens, and best practices for real apps.</p>
          </a>

          <a className="card fade-up" href="/" style={{ animationDelay: "320ms" }}>
            <div className="card-head">Personalization</div>
            <p>Adaptive UIs and user-first experiences.</p>
          </a>

          <a className="card fade-up" href="/" style={{ animationDelay: "400ms" }}>
            <div className="card-head">Urdu Version</div>
            <p>Clear, localized explanations for Urdu readers.</p>
          </a>
        </section>
      </main>

      {/* FOOTER */}
      <footer className="site-footer">
        <div className="footer-inner">
          <div>© 2025 DocuBook • Built with ❤️</div>
          <div className="footer-links">
            <a href="/privacy">Privacy</a>
            <a href="/terms">Terms</a>
          </div>
        </div>
      </footer>
    </div>
  );
}
