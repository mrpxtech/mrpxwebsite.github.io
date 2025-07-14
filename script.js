// Smooth scrolling for navigation links
document.addEventListener('DOMContentLoaded', function() {
    // Get all navigation links
    const navLinks = document.querySelectorAll('.nav-menu a[href^="#"]');
    
    navLinks.forEach(link => {
        link.addEventListener('click', function(e) {
            e.preventDefault();
            
            const targetId = this.getAttribute('href');
            const targetSection = document.querySelector(targetId);
            
            if (targetSection) {
                const navHeight = document.querySelector('.navbar').offsetHeight;
                const targetOffset = targetSection.offsetTop - navHeight;
                
                window.scrollTo({
                    top: targetOffset,
                    behavior: 'smooth'
                });
            }
        });
    });
    
    // Navbar background change on scroll
    const navbar = document.querySelector('.navbar');
    
    window.addEventListener('scroll', function() {
        if (window.scrollY > 50) {
            navbar.style.background = 'rgba(255, 255, 255, 0.98)';
            navbar.style.boxShadow = '0 2px 20px rgba(0, 0, 0, 0.1)';
        } else {
            navbar.style.background = 'rgba(255, 255, 255, 0.95)';
            navbar.style.boxShadow = 'none';
        }
    });
    
    // Intersection Observer for animations
    const observerOptions = {
        threshold: 0.1,
        rootMargin: '0px 0px -50px 0px'
    };
    
    const observer = new IntersectionObserver(function(entries) {
        entries.forEach(entry => {
            if (entry.isIntersecting) {
                entry.target.classList.add('animate-in');
            }
        });
    }, observerOptions);
    
    // Observe feature cards
    const featureCards = document.querySelectorAll('.feature-card');
    featureCards.forEach(card => {
        observer.observe(card);
    });
    
    // Observe safety section
    const safetySection = document.querySelector('.safety-content');
    if (safetySection) {
        observer.observe(safetySection);
    }
    
    // Phone mockup interactions
    const controlJoystick = document.querySelector('.control-joystick');
    const controlButtons = document.querySelectorAll('.control-btn');
    
    if (controlJoystick) {
        // Joystick hover effect
        controlJoystick.addEventListener('mouseenter', function() {
            this.style.transform = 'translate(-50%, -50%) scale(1.1)';
            this.style.boxShadow = '0 6px 20px rgba(0, 0, 0, 0.3)';
        });
        
        controlJoystick.addEventListener('mouseleave', function() {
            this.style.transform = 'translate(-50%, -50%) scale(1)';
            this.style.boxShadow = '0 4px 12px rgba(0, 0, 0, 0.2)';
        });
        
        // Joystick click animation
        controlJoystick.addEventListener('click', function() {
            this.style.transform = 'translate(-50%, -50%) scale(0.95)';
            setTimeout(() => {
                this.style.transform = 'translate(-50%, -50%) scale(1)';
            }, 150);
        });
    }
    
    // Control buttons animations
    controlButtons.forEach((button, index) => {
        button.addEventListener('mouseenter', function() {
            this.style.transform = 'scale(1.2)';
            this.style.boxShadow = '0 4px 12px rgba(0, 0, 0, 0.3)';
        });
        
        button.addEventListener('mouseleave', function() {
            this.style.transform = 'scale(1)';
            this.style.boxShadow = '0 2px 6px rgba(0, 0, 0, 0.2)';
        });
        
        button.addEventListener('click', function() {
            this.style.transform = 'scale(0.9)';
            setTimeout(() => {
                this.style.transform = 'scale(1)';
            }, 150);
        });
        
        // Stagger the animation entrance
        setTimeout(() => {
            button.style.opacity = '1';
            button.style.transform = 'translateY(0)';
        }, 500 + (index * 100));
    });
    
    // Parallax effect for hero section
    const hero = document.querySelector('.hero');
    const heroContent = document.querySelector('.hero-content');
    const phoneImage = document.querySelector('.phone-mockup');
    
    window.addEventListener('scroll', function() {
        const scrolled = window.pageYOffset;
        const rate = scrolled * -0.5;
        
        if (hero && scrolled < window.innerHeight) {
            hero.style.transform = `translateY(${rate}px)`;
        }
    });
    
    // Feature cards stagger animation
    featureCards.forEach((card, index) => {
        card.style.opacity = '0';
        card.style.transform = 'translateY(30px)';
        
        setTimeout(() => {
            card.style.transition = 'opacity 0.6s ease, transform 0.6s ease';
            card.style.opacity = '1';
            card.style.transform = 'translateY(0)';
        }, 200 + (index * 150));
    });
    
    // Safety shield animation
    const safetyShield = document.querySelector('.safety-shield');
    
    if (safetyShield) {
        const observerShield = new IntersectionObserver(function(entries) {
            entries.forEach(entry => {
                if (entry.isIntersecting) {
                    entry.target.style.transform = 'scale(1)';
                    entry.target.style.opacity = '1';
                }
            });
        }, observerOptions);
        
        safetyShield.style.transform = 'scale(0.8)';
        safetyShield.style.opacity = '0.5';
        safetyShield.style.transition = 'transform 0.8s ease, opacity 0.8s ease';
        
        observerShield.observe(safetyShield);
    }
    
    // Download button ripple effect
    const downloadBtn = document.querySelector('.download-btn');
    
    if (downloadBtn) {
        downloadBtn.addEventListener('click', function(e) {
            e.preventDefault();
            
            const ripple = document.createElement('span');
            const rect = this.getBoundingClientRect();
            const size = Math.max(rect.width, rect.height);
            const x = e.clientX - rect.left - size / 2;
            const y = e.clientY - rect.top - size / 2;
            
            ripple.style.width = ripple.style.height = size + 'px';
            ripple.style.left = x + 'px';
            ripple.style.top = y + 'px';
            ripple.classList.add('ripple');
            
            this.appendChild(ripple);
            
            setTimeout(() => {
                ripple.remove();
            }, 600);
        });
    }
    
    // Typed text effect for hero title
    const heroTitle = document.querySelector('.hero-title');
    
    if (heroTitle) {
        const text = heroTitle.innerHTML;
        heroTitle.innerHTML = '';
        
        let i = 0;
        const typeWriter = function() {
            if (i < text.length) {
                heroTitle.innerHTML += text.charAt(i);
                i++;
                setTimeout(typeWriter, 50);
            }
        };
        
        setTimeout(typeWriter, 500);
    }
    
    // Mobile menu toggle (if needed)
    const mobileMenuToggle = document.querySelector('.mobile-menu-toggle');
    const navMenu = document.querySelector('.nav-menu');
    
    if (mobileMenuToggle) {
        mobileMenuToggle.addEventListener('click', function() {
            navMenu.classList.toggle('active');
        });
    }
    
    // Scroll to top functionality
    const scrollToTop = document.createElement('button');
    scrollToTop.innerHTML = '↑';
    scrollToTop.classList.add('scroll-to-top');
    scrollToTop.style.cssText = `
        position: fixed;
        bottom: 30px;
        right: 30px;
        width: 50px;
        height: 50px;
        background: #000;
        color: #fff;
        border: none;
        border-radius: 50%;
        font-size: 20px;
        cursor: pointer;
        opacity: 0;
        transition: opacity 0.3s ease, transform 0.3s ease;
        z-index: 1000;
        display: none;
    `;
    
    document.body.appendChild(scrollToTop);
    
    window.addEventListener('scroll', function() {
        if (window.scrollY > 300) {
            scrollToTop.style.display = 'block';
            setTimeout(() => {
                scrollToTop.style.opacity = '1';
            }, 10);
        } else {
            scrollToTop.style.opacity = '0';
            setTimeout(() => {
                scrollToTop.style.display = 'none';
            }, 300);
        }
    });
    
    scrollToTop.addEventListener('click', function() {
        window.scrollTo({
            top: 0,
            behavior: 'smooth'
        });
    });
    
    scrollToTop.addEventListener('mouseenter', function() {
        this.style.transform = 'scale(1.1)';
    });
    
    scrollToTop.addEventListener('mouseleave', function() {
        this.style.transform = 'scale(1)';
    });
    
    // Signup Form Handling
    const signupForm = document.getElementById('signupForm');
    const loginLink = document.getElementById('loginLink');
    
    if (signupForm) {
        signupForm.addEventListener('submit', handleSignup);
        
        // Password confirmation validation
        const passwordInput = document.getElementById('password');
        const confirmPasswordInput = document.getElementById('confirmPassword');
        
        if (confirmPasswordInput) {
            confirmPasswordInput.addEventListener('input', function() {
                if (this.value !== passwordInput.value) {
                    this.setCustomValidity('Passwords do not match');
                } else {
                    this.setCustomValidity('');
                }
            });
        }
        
        // Real-time form validation
        const formInputs = signupForm.querySelectorAll('input, select');
        formInputs.forEach(input => {
            input.addEventListener('blur', validateField);
            input.addEventListener('input', clearFieldError);
        });
    }
    
    if (loginLink) {
        loginLink.addEventListener('click', function(e) {
            e.preventDefault();
            showLoginModal();
        });
    }
    
    // Pricing plan selection
    const planSelect = document.getElementById('plan');
    if (planSelect) {
        planSelect.addEventListener('change', updatePricingDisplay);
    }
    
    // Check if user is already logged in
    checkAuthStatus();
    
    // Initialize usage tracking
    initializeUsageTracking();
});

// Signup form handler
async function handleSignup(e) {
    e.preventDefault();
    
    const formData = new FormData(e.target);
    const userData = {
        fullName: formData.get('fullName'),
        email: formData.get('email'),
        password: formData.get('password'),
        plan: formData.get('plan'),
        terms: formData.get('terms') === 'on'
    };
    
    // Validate form
    if (!validateSignupForm(userData)) {
        return;
    }
    
    // Show loading state
    const submitBtn = e.target.querySelector('button[type="submit"]');
    const originalText = submitBtn.textContent;
    submitBtn.textContent = 'Creating Account...';
    submitBtn.disabled = true;
    
    try {
        const response = await fetch('/api/auth/signup', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify(userData)
        });
        
        const result = await response.json();
        
        if (response.ok) {
            showSuccessMessage('Account created successfully! Welcome to SurgicalControl.');
            localStorage.setItem('userToken', result.token);
            localStorage.setItem('userData', JSON.stringify(result.user));
            updateUIForLoggedInUser(result.user);
        } else {
            showErrorMessage(result.message || 'Failed to create account. Please try again.');
        }
    } catch (error) {
        console.error('Signup error:', error);
        showErrorMessage('Network error. Please check your connection and try again.');
    } finally {
        submitBtn.textContent = originalText;
        submitBtn.disabled = false;
    }
}

// Form validation
function validateSignupForm(data) {
    const errors = [];
    
    if (!data.fullName || data.fullName.trim().length < 2) {
        errors.push('Full name must be at least 2 characters long');
    }
    
    if (!data.email || !isValidEmail(data.email)) {
        errors.push('Please enter a valid email address');
    }
    
    if (!data.password || data.password.length < 8) {
        errors.push('Password must be at least 8 characters long');
    }
    
    if (!data.plan) {
        errors.push('Please select a plan');
    }
    
    if (!data.terms) {
        errors.push('You must agree to the terms and conditions');
    }
    
    if (errors.length > 0) {
        showErrorMessage(errors.join('<br>'));
        return false;
    }
    
    return true;
}

function validateField(e) {
    const field = e.target;
    const value = field.value.trim();
    
    switch (field.name) {
        case 'fullName':
            if (value.length < 2) {
                showFieldError(field, 'Name must be at least 2 characters');
            }
            break;
        case 'email':
            if (!isValidEmail(value)) {
                showFieldError(field, 'Please enter a valid email');
            }
            break;
        case 'password':
            if (value.length < 8) {
                showFieldError(field, 'Password must be at least 8 characters');
            }
            break;
        case 'confirmPassword':
            const password = document.getElementById('password').value;
            if (value !== password) {
                showFieldError(field, 'Passwords do not match');
            }
            break;
    }
}

function clearFieldError(e) {
    const field = e.target;
    field.classList.remove('error');
    const errorMsg = field.parentNode.querySelector('.error-message');
    if (errorMsg) {
        errorMsg.remove();
    }
}

function showFieldError(field, message) {
    field.classList.add('error');
    let errorMsg = field.parentNode.querySelector('.error-message');
    if (!errorMsg) {
        errorMsg = document.createElement('div');
        errorMsg.className = 'error-message';
        field.parentNode.appendChild(errorMsg);
    }
    errorMsg.textContent = message;
}

// Utility functions
function isValidEmail(email) {
    const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
    return emailRegex.test(email);
}

function showSuccessMessage(message) {
    showNotification(message, 'success');
}

function showErrorMessage(message) {
    showNotification(message, 'error');
}

function showNotification(message, type) {
    const notification = document.createElement('div');
    notification.className = `notification ${type}`;
    notification.innerHTML = `
        <div class="notification-content">
            <span class="notification-message">${message}</span>
            <button class="notification-close">&times;</button>
        </div>
    `;
    
    notification.style.cssText = `
        position: fixed;
        top: 20px;
        right: 20px;
        background: ${type === 'success' ? '#4CAF50' : '#f44336'};
        color: white;
        padding: 1rem 1.5rem;
        border-radius: 8px;
        box-shadow: 0 4px 12px rgba(0,0,0,0.15);
        z-index: 10000;
        max-width: 400px;
        animation: slideInRight 0.3s ease;
    `;
    
    document.body.appendChild(notification);
    
    // Auto remove after 5 seconds
    setTimeout(() => {
        notification.remove();
    }, 5000);
    
    // Close button
    notification.querySelector('.notification-close').addEventListener('click', () => {
        notification.remove();
    });
}

// Login modal
function showLoginModal() {
    const modal = document.createElement('div');
    modal.className = 'login-modal';
    modal.innerHTML = `
        <div class="modal-content">
            <div class="modal-header">
                <h3>Sign In</h3>
                <button class="modal-close">&times;</button>
            </div>
            <form id="loginForm">
                <div class="form-group">
                    <label for="loginEmail">Email</label>
                    <input type="email" id="loginEmail" name="email" required>
                </div>
                <div class="form-group">
                    <label for="loginPassword">Password</label>
                    <input type="password" id="loginPassword" name="password" required>
                </div>
                <button type="submit" class="btn btn-primary btn-full">Sign In</button>
            </form>
            <div class="modal-footer">
                <p>Don't have an account? <a href="#" id="switchToSignup">Sign up</a></p>
            </div>
        </div>
    `;
    
    modal.style.cssText = `
        position: fixed;
        top: 0;
        left: 0;
        width: 100%;
        height: 100%;
        background: rgba(0,0,0,0.5);
        display: flex;
        align-items: center;
        justify-content: center;
        z-index: 10000;
    `;
    
    document.body.appendChild(modal);
    
    // Close modal
    modal.querySelector('.modal-close').addEventListener('click', () => {
        modal.remove();
    });
    
    modal.addEventListener('click', (e) => {
        if (e.target === modal) {
            modal.remove();
        }
    });
    
    // Handle login form
    modal.querySelector('#loginForm').addEventListener('submit', handleLogin);
}

// Login handler
async function handleLogin(e) {
    e.preventDefault();
    
    const formData = new FormData(e.target);
    const loginData = {
        email: formData.get('email'),
        password: formData.get('password')
    };
    
    const submitBtn = e.target.querySelector('button[type="submit"]');
    const originalText = submitBtn.textContent;
    submitBtn.textContent = 'Signing In...';
    submitBtn.disabled = true;
    
    try {
        const response = await fetch('/api/auth/login', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify(loginData)
        });
        
        const result = await response.json();
        
        if (response.ok) {
            localStorage.setItem('userToken', result.token);
            localStorage.setItem('userData', JSON.stringify(result.user));
            updateUIForLoggedInUser(result.user);
            document.querySelector('.login-modal').remove();
            showSuccessMessage('Welcome back!');
        } else {
            showErrorMessage(result.message || 'Invalid credentials');
        }
    } catch (error) {
        console.error('Login error:', error);
        showErrorMessage('Network error. Please try again.');
    } finally {
        submitBtn.textContent = originalText;
        submitBtn.disabled = false;
    }
}

// Check authentication status
function checkAuthStatus() {
    const token = localStorage.getItem('userToken');
    const userData = localStorage.getItem('userData');
    
    if (token && userData) {
        try {
            const user = JSON.parse(userData);
            updateUIForLoggedInUser(user);
        } catch (error) {
            console.error('Error parsing user data:', error);
            localStorage.removeItem('userToken');
            localStorage.removeItem('userData');
        }
    }
}

// Update UI for logged in user
function updateUIForLoggedInUser(user) {
    const signupSection = document.getElementById('signup');
    const navMenu = document.querySelector('.nav-menu');
    
    if (signupSection) {
        signupSection.innerHTML = `
            <div class="container">
                <div class="user-dashboard">
                    <h2>Welcome, ${user.fullName}!</h2>
                    <div class="user-stats">
                        <div class="stat-card">
                            <h3>Usage This Month</h3>
                            <p class="stat-value" id="monthlyUsage">0 seconds</p>
                        </div>
                        <div class="stat-card">
                            <h3>Current Plan</h3>
                            <p class="stat-value">${user.plan}</p>
                        </div>
                        <div class="stat-card">
                            <h3>Balance</h3>
                            <p class="stat-value" id="userBalance">$0.00</p>
                        </div>
                    </div>
                    <div class="dashboard-actions">
                        <button class="btn btn-primary" onclick="downloadApp()">Download App</button>
                        <button class="btn btn-secondary" onclick="viewUsageHistory()">Usage History</button>
                        <button class="btn btn-secondary" onclick="logout()">Sign Out</button>
                    </div>
                </div>
            </div>
        `;
    }
    
    // Update navigation
    if (navMenu) {
        const signupLink = navMenu.querySelector('a[href="#signup"]');
        if (signupLink) {
            signupLink.textContent = 'Dashboard';
            signupLink.href = '#signup';
        }
    }
    
    // Load user data
    loadUserData();
}

// Load user data from API
async function loadUserData() {
    const token = localStorage.getItem('userToken');
    if (!token) return;
    
    try {
        const response = await fetch('/api/user/data', {
            headers: {
                'Authorization': `Bearer ${token}`
            }
        });
        
        if (response.ok) {
            const userData = await response.json();
            updateUserStats(userData);
        }
    } catch (error) {
        console.error('Error loading user data:', error);
    }
}

// Update user statistics
function updateUserStats(data) {
    const monthlyUsage = document.getElementById('monthlyUsage');
    const userBalance = document.getElementById('userBalance');
    
    if (monthlyUsage) {
        monthlyUsage.textContent = `${data.monthlyUsage || 0} seconds`;
    }
    
    if (userBalance) {
        userBalance.textContent = `$${(data.balance || 0).toFixed(2)}`;
    }
}

// Usage tracking
function initializeUsageTracking() {
    let sessionStart = null;
    let isTracking = false;
    
    // Start tracking when user interacts with surgical controls
    const surgicalControls = document.querySelectorAll('.control-joystick, .control-btn');
    surgicalControls.forEach(control => {
        control.addEventListener('click', () => {
            if (!isTracking) {
                startUsageTracking();
            }
        });
    });
    
    function startUsageTracking() {
        if (!localStorage.getItem('userToken')) return;
        
        isTracking = true;
        sessionStart = Date.now();
        
        // Send start session to API
        fetch('/api/usage/start', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
                'Authorization': `Bearer ${localStorage.getItem('userToken')}`
            }
        }).catch(error => console.error('Error starting usage tracking:', error));
    }
    
    // Stop tracking when user leaves page
    window.addEventListener('beforeunload', () => {
        if (isTracking && sessionStart) {
            const duration = Math.floor((Date.now() - sessionStart) / 1000);
            stopUsageTracking(duration);
        }
    });
}

function stopUsageTracking(duration) {
    if (!localStorage.getItem('userToken')) return;
    
    fetch('/api/usage/stop', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
            'Authorization': `Bearer ${localStorage.getItem('userToken')}`
        },
        body: JSON.stringify({ duration })
    }).catch(error => console.error('Error stopping usage tracking:', error));
}

// User actions - Make these global functions
window.downloadApp = function() {
    // Redirect to app download or show download modal
    window.open('https://play.google.com/store/apps/details?id=com.surgicalcontrol.app', '_blank');
};

window.viewUsageHistory = function() {
    // Show usage history modal
    showUsageHistoryModal();
};

window.logout = function() {
    localStorage.removeItem('userToken');
    localStorage.removeItem('userData');
    location.reload();
};

function showUsageHistoryModal() {
    const modal = document.createElement('div');
    modal.className = 'usage-modal';
    modal.innerHTML = `
        <div class="modal-content">
            <div class="modal-header">
                <h3>Usage History</h3>
                <button class="modal-close">&times;</button>
            </div>
            <div class="usage-content">
                <div class="loading">Loading usage history...</div>
            </div>
        </div>
    `;
    
    modal.style.cssText = `
        position: fixed;
        top: 0;
        left: 0;
        width: 100%;
        height: 100%;
        background: rgba(0,0,0,0.5);
        display: flex;
        align-items: center;
        justify-content: center;
        z-index: 10000;
    `;
    
    document.body.appendChild(modal);
    
    // Load usage history
    loadUsageHistory(modal.querySelector('.usage-content'));
    
    // Close modal
    modal.querySelector('.modal-close').addEventListener('click', () => {
        modal.remove();
    });
    
    modal.addEventListener('click', (e) => {
        if (e.target === modal) {
            modal.remove();
        }
    });
}

async function loadUsageHistory(container) {
    const token = localStorage.getItem('userToken');
    if (!token) return;
    
    try {
        const response = await fetch('/api/user/usage', {
            headers: {
                'Authorization': `Bearer ${token}`
            }
        });
        
        if (response.ok) {
            const usageData = await response.json();
            displayUsageHistory(container, usageData);
        } else {
            container.innerHTML = '<p>Error loading usage history</p>';
        }
    } catch (error) {
        console.error('Error loading usage history:', error);
        container.innerHTML = '<p>Error loading usage history</p>';
    }
}

function displayUsageHistory(container, data) {
    if (!data || data.length === 0) {
        container.innerHTML = '<p>No usage history found</p>';
        return;
    }
    
    const usageHTML = data.map(session => `
        <div class="usage-session">
            <div class="session-date">${new Date(session.startTime).toLocaleDateString()}</div>
            <div class="session-duration">${session.duration} seconds</div>
            <div class="session-cost">$${session.cost.toFixed(2)}</div>
        </div>
    `).join('');
    
    container.innerHTML = `
        <div class="usage-summary">
            <h4>Total Usage: ${data.reduce((sum, session) => sum + session.duration, 0)} seconds</h4>
            <h4>Total Cost: $${data.reduce((sum, session) => sum + session.cost, 0).toFixed(2)}</h4>
        </div>
        <div class="usage-sessions">
            ${usageHTML}
        </div>
    `;
}

// Update pricing display
function updatePricingDisplay() {
    const planSelect = document.getElementById('plan');
    const selectedPlan = planSelect.value;
    
    // Update any pricing displays based on selection
    const pricingCards = document.querySelectorAll('.pricing-card');
    pricingCards.forEach(card => {
        card.classList.remove('selected');
        if (card.querySelector('h3').textContent.toLowerCase().includes(selectedPlan)) {
            card.classList.add('selected');
        }
    });
}

// Add CSS for new components
const additionalStyles = `
    .notification {
        animation: slideInRight 0.3s ease;
    }
    
    @keyframes slideInRight {
        from { transform: translateX(100%); opacity: 0; }
        to { transform: translateX(0); opacity: 1; }
    }
    
    .form-group.error input {
        border-color: #f44336;
    }
    
    .error-message {
        color: #f44336;
        font-size: 0.9rem;
        margin-top: 0.5rem;
    }
    
    .user-dashboard {
        text-align: center;
        padding: 2rem;
    }
    
    .user-stats {
        display: grid;
        grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
        gap: 1.5rem;
        margin: 2rem 0;
    }
    
    .stat-card {
        background: #f8f8f8;
        padding: 1.5rem;
        border-radius: 12px;
        text-align: center;
    }
    
    .stat-value {
        font-size: 1.5rem;
        font-weight: 600;
        color: #000;
        margin-top: 0.5rem;
    }
    
    .dashboard-actions {
        display: flex;
        gap: 1rem;
        justify-content: center;
        flex-wrap: wrap;
    }
    
    .modal-content {
        background: white;
        border-radius: 12px;
        padding: 2rem;
        max-width: 400px;
        width: 90%;
        max-height: 80vh;
        overflow-y: auto;
    }
    
    .modal-header {
        display: flex;
        justify-content: space-between;
        align-items: center;
        margin-bottom: 1.5rem;
    }
    
    .modal-close {
        background: none;
        border: none;
        font-size: 1.5rem;
        cursor: pointer;
        color: #666;
    }
    
    .modal-footer {
        text-align: center;
        margin-top: 1.5rem;
        padding-top: 1.5rem;
        border-top: 1px solid #e0e0e0;
    }
    
    .usage-session {
        display: flex;
        justify-content: space-between;
        padding: 1rem;
        border-bottom: 1px solid #e0e0e0;
    }
    
    .usage-summary {
        background: #f8f8f8;
        padding: 1rem;
        border-radius: 8px;
        margin-bottom: 1rem;
    }
    
    .pricing-card.selected {
        border-color: #000;
        transform: scale(1.02);
    }
`;

// Inject additional styles
const styleSheet = document.createElement('style');
styleSheet.textContent = additionalStyles;
document.head.appendChild(styleSheet);