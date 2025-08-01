// Global variables
let API_BASE_URL = 'http://localhost:3000/api';
let isPageReady = false;

// Initialize page
function initializePage() {
    console.log('🔧 Debug page initializing...');
    
    // Update environment info
    updateEnvironmentInfo();
    
    // Add event listeners
    addEventListeners();
    
    // Mark page as ready
    isPageReady = true;
    updatePageStatus('Page ready! Buttons should work now.', 'ready');
    
    console.log('✅ Page initialized successfully');
}

function updateEnvironmentInfo() {
    const envInfo = document.getElementById('envInfo');
    envInfo.textContent = 
        `Current URL: ${window.location.href}\n` +
        `Hostname: ${window.location.hostname}\n` +
        `Port: ${window.location.port}\n` +
        `API Base URL: ${API_BASE_URL}\n` +
        `User Agent: ${navigator.userAgent}\n` +
        `Page Ready: ${isPageReady}`;
}

function updatePageStatus(message, type) {
    const statusDiv = document.getElementById('pageStatus');
    statusDiv.textContent = message;
    statusDiv.className = `status ${type}`;
}

function addEventListeners() {
    console.log('🔗 Adding event listeners...');
    
    // Health check button
    const healthBtn = document.getElementById('healthBtn');
    if (healthBtn) {
        healthBtn.addEventListener('click', testHealth);
        console.log('✅ Health button listener added');
    } else {
        console.error('❌ Health button not found');
    }

    // Login button
    const loginBtn = document.getElementById('loginBtn');
    if (loginBtn) {
        loginBtn.addEventListener('click', testLogin);
        console.log('✅ Login button listener added');
    } else {
        console.error('❌ Login button not found');
    }

    // Signup button
    const signupBtn = document.getElementById('signupBtn');
    if (signupBtn) {
        signupBtn.addEventListener('click', testSignup);
        console.log('✅ Signup button listener added');
    } else {
        console.error('❌ Signup button not found');
    }

    console.log('🔗 All event listeners added');
}

async function testHealth() {
    console.log('🏥 Testing health endpoint...');
    const resultDiv = document.getElementById('healthResult');
    const button = document.getElementById('healthBtn');
    
    button.disabled = true;
    resultDiv.textContent = 'Testing...';
    resultDiv.className = 'result info';

    try {
        console.log('📡 Making request to:', `${API_BASE_URL}/health`);
        const response = await fetch(`${API_BASE_URL}/health`);
        console.log('📡 Response received:', response.status);
        
        const data = await response.json();
        console.log('📡 Data received:', data);
        
        resultDiv.textContent = `✅ Health Check Success\nStatus: ${response.status}\nData: ${JSON.stringify(data, null, 2)}`;
        resultDiv.className = 'result success';
    } catch (error) {
        console.error('❌ Health check failed:', error);
        resultDiv.textContent = `❌ Health Check Failed\nError: ${error.message}\nType: ${error.name}`;
        resultDiv.className = 'result error';
    } finally {
        button.disabled = false;
    }
}

async function testLogin() {
    console.log('🔐 Testing login...');
    const resultDiv = document.getElementById('loginResult');
    const button = document.getElementById('loginBtn');
    const email = document.getElementById('email').value;
    const password = document.getElementById('password').value;

    button.disabled = true;
    resultDiv.textContent = 'Testing login...';
    resultDiv.className = 'result info';

    try {
        console.log('📡 Making login request with:', { email, password: '***' });
        const response = await fetch(`${API_BASE_URL}/auth/login`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({ email, password })
        });

        console.log('📡 Login response status:', response.status);
        const data = await response.json();
        console.log('📡 Login response data:', data);

        if (response.ok) {
            resultDiv.textContent = `✅ Login Success\nStatus: ${response.status}\nToken: ${data.token.substring(0, 50)}...\nUser: ${data.user.fullName}`;
            resultDiv.className = 'result success';
        } else {
            resultDiv.textContent = `❌ Login Failed\nStatus: ${response.status}\nMessage: ${data.message}`;
            resultDiv.className = 'result error';
        }
    } catch (error) {
        console.error('❌ Login error:', error);
        resultDiv.textContent = `❌ Network Error\nError: ${error.message}\nType: ${error.name}`;
        resultDiv.className = 'result error';
    } finally {
        button.disabled = false;
    }
}

async function testSignup() {
    console.log('📝 Testing signup...');
    const resultDiv = document.getElementById('loginResult');
    const button = document.getElementById('signupBtn');
    const email = `test${Date.now()}@example.com`;
    const password = 'password123';

    button.disabled = true;
    resultDiv.textContent = 'Testing signup...';
    resultDiv.className = 'result info';

    try {
        console.log('📡 Making signup request with:', { email, password: '***' });
        const response = await fetch(`${API_BASE_URL}/auth/signup`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({ 
                fullName: 'Test User', 
                email, 
                password, 
                plan: 'basic' 
            })
        });

        console.log('📡 Signup response status:', response.status);
        const data = await response.json();
        console.log('📡 Signup response data:', data);

        if (response.ok) {
            resultDiv.textContent = `✅ Signup Success\nStatus: ${response.status}\nUser: ${data.user.fullName}\nEmail: ${data.user.email}`;
            resultDiv.className = 'result success';
        } else {
            resultDiv.textContent = `❌ Signup Failed\nStatus: ${response.status}\nMessage: ${data.message}`;
            resultDiv.className = 'result error';
        }
    } catch (error) {
        console.error('❌ Signup error:', error);
        resultDiv.textContent = `❌ Network Error\nError: ${error.message}\nType: ${error.name}`;
        resultDiv.className = 'result error';
    } finally {
        button.disabled = false;
    }
}

// Initialize when DOM is ready
if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', initializePage);
} else {
    initializePage();
}

// Also try on window load as backup
window.addEventListener('load', () => {
    if (!isPageReady) {
        console.log('🔄 Window loaded, initializing...');
        initializePage();
    }
});

console.log('📄 Debug page script loaded'); 