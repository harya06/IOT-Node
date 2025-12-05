/* ============================================================
   IoT Node - Blue-Tech Theme
   JavaScript Interactions & Utilities
   ============================================================ */

(function() {
  'use strict';

  // ============================================================
  // DOM Ready Handler
  // ============================================================
  document.addEventListener('DOMContentLoaded', function() {
    initPasswordToggles();
    initFormValidation();
    initSmoothTransitions();
    initPageAnimations();
    initTooltips();
  });

  // ============================================================
  // Password Toggle Functionality
  // ============================================================
  function initPasswordToggles() {
    const toggleButtons = document.querySelectorAll('.password-toggle');
    
    toggleButtons.forEach(function(btn) {
      btn.addEventListener('click', function() {
        const wrapper = this.closest('.input-wrapper');
        const input = wrapper.querySelector('input');
        
        if (input.type === 'password') {
          input.type = 'text';
          this.textContent = '🙈';
          this.setAttribute('aria-label', 'Hide password');
        } else {
          input.type = 'password';
          this.textContent = '👁️';
          this.setAttribute('aria-label', 'Show password');
        }
        
        // Add subtle animation
        this.style.transform = 'translateY(-50%) scale(1.2)';
        setTimeout(() => {
          this.style.transform = 'translateY(-50%) scale(1)';
        }, 150);
      });
    });
  }

  // ============================================================
  // Form Validation
  // ============================================================
  function initFormValidation() {
    const forms = document.querySelectorAll('form');
    
    forms.forEach(function(form) {
      // Real-time validation on input
      const inputs = form.querySelectorAll('input[required], select[required]');
      
      inputs.forEach(function(input) {
        input.addEventListener('blur', function() {
          validateField(this);
        });
        
        input.addEventListener('input', function() {
          // Remove error state on typing
          if (this.classList.contains('error')) {
            this.classList.remove('error');
            removeFieldError(this);
          }
        });
      });
      
      // Form submission validation
      form.addEventListener('submit', function(e) {
        let isValid = true;
        
        inputs.forEach(function(input) {
          if (!validateField(input)) {
            isValid = false;
          }
        });
        
        if (!isValid) {
          e.preventDefault();
          showAlert('Please fill in all required fields correctly.', 'error');
          
          // Focus first invalid field
          const firstError = form.querySelector('.error');
          if (firstError) {
            firstError.focus();
          }
        } else {
          // Show loading state
          const submitBtn = form.querySelector('button[type="submit"]');
          if (submitBtn) {
            submitBtn.disabled = true;
            submitBtn.innerHTML = '<span class="spinner-small"></span> Processing...';
          }
        }
      });
    });
  }

  function validateField(field) {
    const value = field.value.trim();
    let isValid = true;
    let errorMessage = '';
    
    // Required check
    if (field.hasAttribute('required') && !value) {
      isValid = false;
      errorMessage = 'This field is required';
    }
    
    // Email validation
    if (field.type === 'email' && value) {
      const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
      if (!emailRegex.test(value)) {
        isValid = false;
        errorMessage = 'Please enter a valid email address';
      }
    }
    
    // Number validation
    if (field.type === 'number' && value) {
      const num = parseInt(value);
      const min = field.getAttribute('min');
      const max = field.getAttribute('max');
      
      if (min && num < parseInt(min)) {
        isValid = false;
        errorMessage = `Value must be at least ${min}`;
      }
      if (max && num > parseInt(max)) {
        isValid = false;
        errorMessage = `Value must be at most ${max}`;
      }
    }
    
    // Port validation (special case)
    if (field.name === 'serverPort' && value) {
      const port = parseInt(value);
      if (port < 1 || port > 65535) {
        isValid = false;
        errorMessage = 'Port must be between 1 and 65535';
      }
    }
    
    // IP Address validation
    if (field.name === 'serverIP' && value) {
      // Allow both IP addresses and hostnames
      const ipRegex = /^(\d{1,3}\.){3}\d{1,3}$/;
      const hostnameRegex = /^[a-zA-Z0-9][a-zA-Z0-9-._]+[a-zA-Z0-9]$/;
      
      if (!ipRegex.test(value) && !hostnameRegex.test(value)) {
        isValid = false;
        errorMessage = 'Please enter a valid IP address or hostname';
      }
    }
    
    // Apply validation state
    if (isValid) {
      field.classList.remove('error');
      removeFieldError(field);
    } else {
      field.classList.add('error');
      showFieldError(field, errorMessage);
    }
    
    return isValid;
  }

  function showFieldError(field, message) {
    removeFieldError(field);
    
    const errorDiv = document.createElement('div');
    errorDiv.className = 'field-error';
    errorDiv.textContent = message;
    errorDiv.style.cssText = `
      color: var(--danger);
      font-size: 0.8125rem;
      margin-top: 0.375rem;
      animation: slideDown 0.2s ease;
    `;
    
    field.parentNode.appendChild(errorDiv);
    field.style.borderColor = 'var(--danger)';
  }

  function removeFieldError(field) {
    const wrapper = field.parentNode;
    const existingError = wrapper.querySelector('.field-error');
    if (existingError) {
      existingError.remove();
    }
    field.style.borderColor = '';
  }

  // ============================================================
  // Alert System
  // ============================================================
  window.showAlert = function(message, type) {
    type = type || 'info';
    
    // Check for existing alert element
    let alertEl = document.getElementById('alert');
    
    if (!alertEl) {
      // Create alert element if it doesn't exist
      alertEl = document.createElement('div');
      alertEl.id = 'alert';
      alertEl.className = 'alert';
      
      const form = document.querySelector('form');
      if (form) {
        form.insertBefore(alertEl, form.firstChild);
      } else {
        document.body.insertBefore(alertEl, document.body.firstChild);
      }
    }
    
    // Set content and show
    alertEl.className = `alert ${type}`;
    alertEl.innerHTML = `<strong>${type === 'error' ? '⚠️ Error' : type === 'success' ? '✅ Success' : 'ℹ️ Info'}</strong><br>${message}`;
    alertEl.classList.add('show');
    
    // Auto hide after 5 seconds
    setTimeout(function() {
      alertEl.classList.remove('show');
    }, 5000);
  };

  // ============================================================
  // Toast Notifications
  // ============================================================
  window.showToast = function(message, duration) {
    duration = duration || 3000;
    
    let toast = document.getElementById('toast');
    
    if (!toast) {
      toast = document.createElement('div');
      toast.id = 'toast';
      toast.className = 'toast';
      toast.innerHTML = `
        <div class="toast-icon">✓</div>
        <div class="toast-text">${message}</div>
      `;
      document.body.appendChild(toast);
    } else {
      toast.querySelector('.toast-text').textContent = message;
    }
    
    toast.classList.add('show');
    
    setTimeout(function() {
      toast.classList.remove('show');
    }, duration);
  };

  // ============================================================
  // Smooth Page Transitions
  // ============================================================
  function initSmoothTransitions() {
    // Add transition class to body
    document.body.style.opacity = '0';
    document.body.style.transition = 'opacity 0.3s ease';
    
    requestAnimationFrame(function() {
      document.body.style.opacity = '1';
    });
    
    // Handle link clicks for smooth exit
    const links = document.querySelectorAll('a[href]:not([href^="#"]):not([href^="javascript"])');
    
    links.forEach(function(link) {
      link.addEventListener('click', function(e) {
        const href = this.getAttribute('href');
        
        // Skip if modifier key pressed or download link
        if (e.metaKey || e.ctrlKey || this.hasAttribute('download')) {
          return;
        }
        
        e.preventDefault();
        document.body.style.opacity = '0';
        
        setTimeout(function() {
          window.location.href = href;
        }, 200);
      });
    });
  }

  // ============================================================
  // Page Entry Animations
  // ============================================================
  function initPageAnimations() {
    // Animate cards on scroll
    const observerOptions = {
      threshold: 0.1,
      rootMargin: '0px 0px -50px 0px'
    };
    
    const observer = new IntersectionObserver(function(entries) {
      entries.forEach(function(entry) {
        if (entry.isIntersecting) {
          entry.target.classList.add('animate-in');
          observer.unobserve(entry.target);
        }
      });
    }, observerOptions);
    
    // Observe elements
    const animateElements = document.querySelectorAll('.card, .stat-card, .output-card, .info-item');
    animateElements.forEach(function(el, index) {
      el.style.opacity = '0';
      el.style.transform = 'translateY(20px)';
      el.style.transition = `opacity 0.4s ease ${index * 0.05}s, transform 0.4s ease ${index * 0.05}s`;
      observer.observe(el);
    });
    
    // Add animate-in styles
    const style = document.createElement('style');
    style.textContent = `
      .animate-in {
        opacity: 1 !important;
        transform: translateY(0) !important;
      }
    `;
    document.head.appendChild(style);
  }

  // ============================================================
  // Tooltip System
  // ============================================================
  function initTooltips() {
    const tooltipElements = document.querySelectorAll('[data-tooltip]');
    
    tooltipElements.forEach(function(el) {
      el.addEventListener('mouseenter', function(e) {
        const text = this.getAttribute('data-tooltip');
        
        const tooltip = document.createElement('div');
        tooltip.className = 'tooltip';
        tooltip.textContent = text;
        tooltip.style.cssText = `
          position: fixed;
          background: var(--bg-card);
          color: var(--text-primary);
          padding: 0.5rem 0.75rem;
          border-radius: var(--border-radius-sm);
          font-size: 0.8125rem;
          box-shadow: var(--shadow-lg);
          border: 1px solid var(--surface-border);
          z-index: 10000;
          pointer-events: none;
          animation: fadeIn 0.2s ease;
        `;
        
        document.body.appendChild(tooltip);
        
        // Position tooltip
        const rect = this.getBoundingClientRect();
        tooltip.style.left = `${rect.left + (rect.width / 2) - (tooltip.offsetWidth / 2)}px`;
        tooltip.style.top = `${rect.top - tooltip.offsetHeight - 8}px`;
        
        this._tooltip = tooltip;
      });
      
      el.addEventListener('mouseleave', function() {
        if (this._tooltip) {
          this._tooltip.remove();
          this._tooltip = null;
        }
      });
    });
  }

  // ============================================================
  // Loading Overlay
  // ============================================================
  window.showLoading = function() {
    let overlay = document.getElementById('loading-overlay');
    
    if (!overlay) {
      overlay = document.createElement('div');
      overlay.id = 'loading-overlay';
      overlay.className = 'loading-overlay';
      overlay.innerHTML = '<div class="spinner"></div>';
      document.body.appendChild(overlay);
    }
    
    overlay.classList.add('show');
  };

  window.hideLoading = function() {
    const overlay = document.getElementById('loading-overlay');
    if (overlay) {
      overlay.classList.remove('show');
    }
  };

  // ============================================================
  // Utility Functions
  // ============================================================
  
  // Debounce function for performance
  window.debounce = function(func, wait) {
    let timeout;
    return function executedFunction() {
      const context = this;
      const args = arguments;
      clearTimeout(timeout);
      timeout = setTimeout(function() {
        func.apply(context, args);
      }, wait);
    };
  };

  // Format date/time
  window.formatDateTime = function(date) {
    if (!(date instanceof Date)) {
      date = new Date(date);
    }
    return date.toLocaleString('en-US', {
      year: 'numeric',
      month: 'short',
      day: 'numeric',
      hour: '2-digit',
      minute: '2-digit'
    });
  };

  // Copy to clipboard
  window.copyToClipboard = function(text) {
    if (navigator.clipboard && navigator.clipboard.writeText) {
      navigator.clipboard.writeText(text).then(function() {
        showToast('Copied to clipboard!');
      }).catch(function() {
        fallbackCopy(text);
      });
    } else {
      fallbackCopy(text);
    }
  };

  function fallbackCopy(text) {
    const textarea = document.createElement('textarea');
    textarea.value = text;
    textarea.style.position = 'fixed';
    textarea.style.opacity = '0';
    document.body.appendChild(textarea);
    textarea.select();
    
    try {
      document.execCommand('copy');
      showToast('Copied to clipboard!');
    } catch (err) {
      showToast('Failed to copy', 'error');
    }
    
    document.body.removeChild(textarea);
  }

  // ============================================================
  // Network Status Indicator
  // ============================================================
  window.addEventListener('online', function() {
    showToast('Connection restored', 2000);
    updateNetworkStatus(true);
  });

  window.addEventListener('offline', function() {
    showAlert('You are offline. Some features may not work.', 'error');
    updateNetworkStatus(false);
  });

  function updateNetworkStatus(isOnline) {
    const statusBadges = document.querySelectorAll('.status-badge');
    statusBadges.forEach(function(badge) {
      if (isOnline) {
        badge.classList.remove('offline');
      } else {
        badge.classList.add('offline');
      }
    });
  }

  // ============================================================
  // Keyboard Shortcuts
  // ============================================================
  document.addEventListener('keydown', function(e) {
    // Escape to close modals/overlays
    if (e.key === 'Escape') {
      const overlay = document.getElementById('loading-overlay');
      if (overlay && overlay.classList.contains('show')) {
        // Don't close loading overlay with escape
        return;
      }
    }
    
    // Ctrl/Cmd + S to save (on config page)
    if ((e.ctrlKey || e.metaKey) && e.key === 's') {
      const form = document.querySelector('form[action="/save-config"]');
      if (form) {
        e.preventDefault();
        form.dispatchEvent(new Event('submit'));
      }
    }
  });

  // ============================================================
  // Auto-refresh for Status Page
  // ============================================================
  if (window.location.pathname === '/status') {
    // Optional: Auto-refresh status every 30 seconds
    // setInterval(function() {
    //   window.location.reload();
    // }, 30000);
  }

})();