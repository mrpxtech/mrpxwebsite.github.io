# SurgicalControl - da Vinci Robot Control App

A comprehensive web application for controlling da Vinci surgical robots through Android devices, featuring real-time monitoring, user management, and usage-based billing.

## 🚀 New Features (Latest Update)

### Enhanced Visual Design
- **Surgical Robot Visualization**: Detailed 3D surgical robot display inside the phone mockup
- **Animated Background Scene**: Animated da Vinci robot arms operating on a patient
- **Precision Indicators**: Real-time precision dots showing surgical accuracy
- **Interactive Controls**: Haptic feedback simulation for surgical control interface

### User Management System
- **User Registration**: Complete signup system with plan selection
- **Secure Authentication**: JWT-based login with bcrypt password hashing
- **Personalized Dashboard**: User-specific usage statistics and account management
- **Usage History**: Detailed session logs with cost breakdown

### Usage-Based Billing
- **Per-Second Billing**: Pay only for actual usage time
- **Three Plan Tiers**:
  - **Basic**: $0.15/second - Standard da Vinci control
  - **Professional**: $0.25/second - Advanced AI assistance
  - **Enterprise**: $0.35/second - Full AI suite with maximum features
- **Real-Time Tracking**: Automatic usage monitoring during surgical control interaction
- **Monthly Analytics**: Comprehensive usage and cost reporting

## 🏥 Features

### Core Functionality
- **Intuitive Control**: Natural touch gestures translate to precise robotic movements
- **AI-Assisted Precision**: Machine learning algorithms provide real-time assistance
- **Real-time Monitoring**: Comprehensive vital signs and system status monitoring
- **Safety Protocols**: FDA-approved control protocols with multiple failsafes

### Technical Features
- **Azure Integration**: Real-time VM monitoring and cost tracking
- **Secure API**: RESTful API with JWT authentication
- **Database Management**: SQLite database for user and usage data
- **Responsive Design**: Mobile-first design for all devices

## 🛠️ Technology Stack

### Frontend
- **HTML5/CSS3**: Modern, responsive design with CSS Grid and Flexbox
- **Vanilla JavaScript**: Interactive features and API integration
- **SF Pro Display Font**: Apple-style typography for professional appearance

### Backend
- **Node.js**: Server-side JavaScript runtime
- **Express.js**: Web application framework
- **SQLite**: Lightweight database for user management
- **JWT**: JSON Web Tokens for authentication
- **bcrypt**: Password hashing for security

### Azure Integration
- **Azure SDK**: Compute, Monitor, and Consumption clients
- **Service Principal**: Secure authentication for Azure resources
- **Real-time Metrics**: CPU, memory, disk, and network monitoring

## 📱 User Interface

### Hero Section
- **Surgical Robot Display**: Detailed visualization of da Vinci robot inside phone
- **Animated Background**: Operating room scene with robot arms
- **Call-to-Action**: Direct signup and download buttons

### Pricing Section
- **Transparent Pricing**: Clear per-second rates for each plan
- **Feature Comparison**: Detailed breakdown of plan benefits
- **Free Trial**: 30-day trial with no setup fees

### User Dashboard
- **Usage Statistics**: Monthly usage and cost tracking
- **Account Management**: Plan details and balance information
- **Session History**: Complete usage logs with timestamps

## 🔐 Security Features

### Authentication
- **JWT Tokens**: Secure session management with 7-day expiration
- **Password Hashing**: bcrypt with salt rounds for password security
- **Input Validation**: Comprehensive form validation and sanitization

### Data Protection
- **HTTPS Ready**: Secure communication protocols
- **Rate Limiting**: API protection against abuse
- **CORS Configuration**: Controlled cross-origin requests

## 📊 Database Schema

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

### Usage Tracking
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

## 🚀 Quick Start

### Prerequisites
- Node.js 18+ 
- npm or yarn
- Azure subscription (for VM monitoring)

### Installation

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd mprxwebsite
   ```

2. **Install dependencies**
   ```bash
   npm install
   ```

3. **Configure environment**
   ```bash
   cp env.example .env
   # Edit .env with your Azure credentials and JWT secret
   ```

4. **Start the server**
   ```bash
   npm start
   ```

5. **Access the application**
   - Website: http://localhost:3000
   - API Health: http://localhost:3000/api/health

## 📋 API Endpoints

### Authentication
- `POST /api/auth/signup` - User registration
- `POST /api/auth/login` - User authentication
- `GET /api/user/data` - Get user data and statistics

### Usage Tracking
- `POST /api/usage/start` - Start usage session
- `POST /api/usage/stop` - End usage session and calculate cost
- `GET /api/user/usage` - Get usage history

### Azure Monitoring
- `GET /api/vms` - Get all Azure VMs with metrics
- `GET /api/costs` - Get Azure cost data
- `POST /api/vms/:rg/:vm/start` - Start VM
- `POST /api/vms/:rg/:vm/stop` - Stop VM
- `POST /api/vms/:rg/:vm/restart` - Restart VM

## 💰 Billing System

### Usage Calculation
- **Real-time Tracking**: Monitors user interaction with surgical controls
- **Per-Second Billing**: Charges based on actual usage time
- **Plan-based Rates**: Different pricing tiers for different feature sets

### Example Costs
- **1 minute on Basic**: $9.00
- **1 minute on Professional**: $15.00
- **1 minute on Enterprise**: $21.00

## 🎨 Design Features

### Visual Elements
- **Surgical Robot Visualization**: Detailed 3D representation with animated components
- **Precision Indicators**: Pulsing green dots showing surgical accuracy
- **Control Interface**: Realistic surgical control panel with joystick and buttons
- **Background Animation**: Subtle robot arm movements in operating room scene

### Responsive Design
- **Mobile-First**: Optimized for mobile devices
- **Tablet Support**: Enhanced layout for tablet screens
- **Desktop Experience**: Full-featured desktop interface

## 🔧 Development

### Project Structure
```
mprxwebsite/
├── index.html          # Main website
├── styles.css          # Styling and animations
├── script.js           # Frontend functionality
├── server.js           # Backend API server
├── package.json        # Dependencies
├── users.db           # SQLite database (auto-generated)
├── admin.html         # Admin dashboard
├── admin-login.html   # Admin authentication
└── docs/              # Documentation
```

### Key Files
- **index.html**: Main website with surgical robot imagery
- **server.js**: Complete backend with user management
- **styles.css**: Surgical robot animations and responsive design
- **script.js**: User authentication and usage tracking

## 📈 Monitoring and Analytics

### User Metrics
- Total registered users by plan
- Monthly usage patterns
- Revenue per plan tier
- User engagement statistics

### Azure Integration
- Real-time VM performance monitoring
- Cost tracking and optimization
- Resource utilization analytics

## 🚀 Deployment

### Production Setup
1. **Environment Configuration**
   - Set production JWT secret
   - Configure Azure credentials
   - Enable HTTPS

2. **Database Setup**
   - SQLite database is auto-created
   - Consider PostgreSQL for large scale

3. **Security Measures**
   - Change default secrets
   - Enable rate limiting
   - Configure CORS properly

### GitHub Pages Deployment
- Automatic deployment via GitHub Actions
- Custom domain support (mrpxtech.com)
- Static file serving with API proxy

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## 📄 License

This project is licensed under the MIT License - see the LICENSE file for details.

## 🆘 Support

For technical support or questions:
- Check the documentation in `/docs`
- Review the API endpoints
- Test the health check endpoint

## 🔮 Future Enhancements

### Planned Features
- **Payment Integration**: Stripe/PayPal integration
- **Advanced Analytics**: Machine learning insights
- **Mobile App**: Native Android application
- **Multi-language Support**: Internationalization
- **Advanced Security**: Two-factor authentication

### Technical Improvements
- **Database Migration**: PostgreSQL for scalability
- **Caching Layer**: Redis for performance
- **Load Balancing**: Multiple server instances
- **Monitoring**: Advanced logging and alerting

---

**SurgicalControl** - Advancing surgical precision through innovative technology.
