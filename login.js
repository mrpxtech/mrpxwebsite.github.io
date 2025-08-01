// Determine API base URL based on current location
let API_BASE_URL;
if (window.location.hostname === 'localhost') {
    // If accessing from localhost, use the same port as the current page
    const currentPort = window.location.port || '80';
    API_BASE_URL = `http://localhost:3000/api`;
} else {
    API_BASE_URL = 'https://api.mrpxtech.com/api';
}

console.log('API Base URL:', API_BASE_URL);
console.log('Current location:', window.location.href);

const loginForm = document.getElementById('loginForm');
const loginBtn = document.getElementById('loginBtn');
const loginText = document.getElementById('loginText');
const loginLoading = document.getElementById('loginLoading');
const errorMessage = document.getElementById('errorMessage');
const successMessage = document.getElementById('successMessage');

function showError(message) {
    errorMessage.textContent = message;
    errorMessage.style.display = 'block';
    successMessage.style.display = 'none';
}

function showSuccess(message) {
    successMessage.textContent = message;
    successMessage.style.display = 'block';
    errorMessage.style.display = 'none';
}

function setLoading(loading) {
    if (loading) {
        loginBtn.disabled = true;
        loginText.style.display = 'none';
        loginLoading.style.display = 'inline-block';
    } else {
        loginBtn.disabled = false;
        loginText.style.display = 'inline';
        loginLoading.style.display = 'none';
    }
}

loginForm.addEventListener('submit', async (e) => {
    e.preventDefault();
    
    const email = document.getElementById('email').value;
    const password = document.getElementById('password').value;

    if (!email || !password) {
        showError('Please fill in all fields');
        return;
    }

    setLoading(true);
    errorMessage.style.display = 'none';

    try {
        console.log('Attempting login to:', `${API_BASE_URL}/auth/login`);
        console.log('Request payload:', { email, password: '***' });
        
        const response = await fetch(`${API_BASE_URL}/auth/login`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({ email, password })
        });

        console.log('Response status:', response.status);
        console.log('Response headers:', response.headers);

        const data = await response.json();
        console.log('Response data:', data);

        if (response.ok) {
            showSuccess('Login successful! Redirecting...');
            
            // Store token and user data
            localStorage.setItem('userToken', data.token);
            localStorage.setItem('userData', JSON.stringify(data.user));
            
            // Redirect to main app or dashboard
            setTimeout(() => {
                window.location.href = 'index.html';
            }, 1500);
        } else {
            showError(data.message || 'Login failed');
        }
    } catch (error) {
        console.error('Login error:', error);
        console.error('Error details:', {
            name: error.name,
            message: error.message,
            stack: error.stack
        });
        showError(`Network error: ${error.message}. Please check your connection and try again.`);
    } finally {
        setLoading(false);
    }
});

// Check if user is already logged in
window.addEventListener('load', () => {
    const token = localStorage.getItem('userToken');
    if (token) {
        showSuccess('Already logged in. Redirecting...');
        setTimeout(() => {
            window.location.href = 'index.html';
        }, 1500);
    }
}); 