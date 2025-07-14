# SurgicalControl User Management System

This document describes the user management and usage tracking system for the SurgicalControl application.

## Features

### User Authentication
- **Signup**: Users can create accounts with email, password, and plan selection
- **Login**: Secure authentication with JWT tokens
- **Password Security**: Passwords are hashed using bcrypt
- **Session Management**: JWT tokens with 7-day expiration

### Usage-Based Billing
- **Per-Second Billing**: Users are charged based on actual usage time
- **Plan Tiers**: 
  - Basic: $0.15/second
  - Professional: $0.25/second  
  - Enterprise: $0.35/second
- **Real-time Tracking**: Usage is tracked when users interact with surgical controls
- **Usage History**: Complete history of all sessions with costs

### User Dashboard
- **Usage Statistics**: Monthly usage and costs
- **Account Balance**: Current balance tracking
- **Plan Management**: View and manage subscription plans
- **Session History**: Detailed usage session logs

## Database Schema

### Users Table
```sql
CREATE TABLE users (
    id TEXT PRIMARY KEY,
    fullName TEXT NOT NULL,
    email TEXT UNIQUE NOT NULL,
    password TEXT NOT NULL,
    plan TEXT NOT NULL,
    balance REAL DEFAULT 0.0,
    createdAt DATETIME DEFAULT CURRENT_TIMESTAMP,
    updatedAt DATETIME DEFAULT CURRENT_TIMESTAMP
);
```

### Usage Sessions Table
```sql
CREATE TABLE usage_sessions (
    id TEXT PRIMARY KEY,
    userId TEXT NOT NULL,
    startTime DATETIME DEFAULT CURRENT_TIMESTAMP,
    endTime DATETIME,
    duration INTEGER DEFAULT 0,
    cost REAL DEFAULT 0.0,
    FOREIGN KEY (userId) REFERENCES users (id)
);
```

### Monthly Usage Table
```sql
CREATE TABLE monthly_usage (
    id TEXT PRIMARY KEY,
    userId TEXT NOT NULL,
    year INTEGER NOT NULL,
    month INTEGER NOT NULL,
    totalSeconds INTEGER DEFAULT 0,
    totalCost REAL DEFAULT 0.0,
    FOREIGN KEY (userId) REFERENCES users (id),
    UNIQUE(userId, year, month)
);
```

## API Endpoints

### Authentication
- `POST /api/auth/signup` - Create new user account
- `POST /api/auth/login` - User login
- `GET /api/user/data` - Get user data and statistics

### Usage Tracking
- `POST /api/usage/start` - Start usage session
- `POST /api/usage/stop` - End usage session and calculate cost
- `GET /api/user/usage` - Get usage history

### Request/Response Examples

#### Signup
```json
POST /api/auth/signup
{
    "fullName": "Dr. John Smith",
    "email": "john.smith@hospital.com",
    "password": "securepassword123",
    "plan": "professional"
}

Response:
{
    "message": "User created successfully",
    "token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
    "user": {
        "id": "uuid-here",
        "fullName": "Dr. John Smith",
        "email": "john.smith@hospital.com",
        "plan": "professional"
    }
}
```

#### Login
```json
POST /api/auth/login
{
    "email": "john.smith@hospital.com",
    "password": "securepassword123"
}

Response:
{
    "message": "Login successful",
    "token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
    "user": {
        "id": "uuid-here",
        "fullName": "Dr. John Smith",
        "email": "john.smith@hospital.com",
        "plan": "professional"
    }
}
```

#### Usage Session
```json
POST /api/usage/start
Headers: Authorization: Bearer <token>

Response:
{
    "message": "Usage session started",
    "sessionId": "session-uuid"
}

POST /api/usage/stop
Headers: Authorization: Bearer <token>
{
    "duration": 120
}

Response:
{
    "message": "Usage session recorded",
    "duration": 120,
    "cost": "30.00"
}
```

## Frontend Integration

### User Interface Features
1. **Signup Form**: Complete registration with plan selection
2. **Login Modal**: Secure authentication interface
3. **User Dashboard**: Shows usage statistics and account info
4. **Usage History**: Modal with detailed session logs
5. **Real-time Tracking**: Automatic usage tracking on surgical control interaction

### JavaScript Functions
- `handleSignup()` - Process user registration
- `handleLogin()` - Process user authentication
- `updateUIForLoggedInUser()` - Update interface for authenticated users
- `initializeUsageTracking()` - Start usage monitoring
- `showUsageHistoryModal()` - Display usage history

## Security Features

### Password Security
- Passwords are hashed using bcrypt with salt rounds of 10
- No plain text passwords are stored in the database

### JWT Authentication
- Tokens expire after 7 days
- Secure token verification middleware
- Protected API endpoints

### Input Validation
- Email format validation
- Password strength requirements (minimum 8 characters)
- Plan validation against allowed options

## Setup Instructions

### 1. Install Dependencies
```bash
npm install
```

### 2. Environment Configuration
Copy `env.example` to `.env` and configure:
```bash
cp env.example .env
```

Required environment variables:
- `JWT_SECRET`: Secret key for JWT token signing
- `AZURE_SUBSCRIPTION_ID`: Azure subscription ID
- `AZURE_TENANT_ID`: Azure tenant ID
- `AZURE_CLIENT_ID`: Azure client ID
- `AZURE_CLIENT_SECRET`: Azure client secret

### 3. Start the Server
```bash
npm start
```

The database will be automatically created as `users.db` in the project root.

## Usage Tracking Implementation

### How Usage is Tracked
1. **Session Start**: When user interacts with surgical controls (joystick/buttons)
2. **Session End**: When user leaves the page or stops interaction
3. **Cost Calculation**: Duration × Plan Rate per second

### Example Usage Scenarios
- **Basic Plan**: 60 seconds = $9.00
- **Professional Plan**: 60 seconds = $15.00
- **Enterprise Plan**: 60 seconds = $21.00

## Billing and Payment

### Current Implementation
- Usage tracking and cost calculation
- Monthly usage aggregation
- Balance tracking in database

### Future Enhancements
- Integration with payment processors (Stripe, PayPal)
- Automatic billing cycles
- Invoice generation
- Payment method management

## Monitoring and Analytics

### Available Metrics
- Total users by plan
- Monthly usage patterns
- Revenue per plan tier
- User engagement metrics

### Database Queries
```sql
-- Total users by plan
SELECT plan, COUNT(*) as user_count FROM users GROUP BY plan;

-- Monthly revenue
SELECT year, month, SUM(totalCost) as revenue 
FROM monthly_usage 
GROUP BY year, month 
ORDER BY year DESC, month DESC;

-- Top users by usage
SELECT u.fullName, mu.totalSeconds, mu.totalCost
FROM users u
JOIN monthly_usage mu ON u.id = mu.userId
WHERE mu.year = 2024 AND mu.month = 12
ORDER BY mu.totalCost DESC
LIMIT 10;
```

## Troubleshooting

### Common Issues

1. **Database Connection Errors**
   - Ensure SQLite is installed
   - Check file permissions for `users.db`

2. **JWT Token Issues**
   - Verify `JWT_SECRET` is set in environment
   - Check token expiration

3. **Usage Tracking Not Working**
   - Verify user is authenticated
   - Check browser console for errors
   - Ensure surgical controls are properly initialized

### Debug Mode
Enable debug logging by setting:
```bash
DEBUG=app:*
npm start
```

## Production Considerations

### Security
- Change default JWT secret
- Use HTTPS in production
- Implement rate limiting
- Add request validation middleware

### Performance
- Consider database indexing for large datasets
- Implement caching for frequently accessed data
- Monitor database performance

### Scalability
- Consider migrating to PostgreSQL for larger scale
- Implement connection pooling
- Add load balancing for high traffic

## Support

For technical support or questions about the user management system, please refer to the main project documentation or contact the development team. 