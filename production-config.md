# Production Configuration for mrpxtech.com

## 🌐 Domain Configuration

### DNS Settings
Configure your DNS provider with these records:

```
# Main website
A     @           YOUR_SERVER_IP
CNAME www         mrpxtech.com

# API subdomain
A     api         YOUR_API_SERVER_IP
# OR if using Azure App Service
CNAME api         surgicalcontrol-admin.azurewebsites.net

# Email (if needed)
MX    @           mail.mrpxtech.com
TXT   @           v=spf1 include:_spf.google.com ~all
```

### SSL Certificate
- **Let's Encrypt**: Free SSL certificates
- **Cloudflare**: Free SSL with CDN
- **Azure**: Managed certificates for App Service

## 🚀 Deployment Options

### Option 1: Traditional Web Server

#### Apache Configuration
```apache
<VirtualHost *:80>
    ServerName mrpxtech.com
    ServerAlias www.mrpxtech.com
    DocumentRoot /var/www/mrpxtech.com/_site
    
    <Directory /var/www/mrpxtech.com/_site>
        AllowOverride All
        Require all granted
    </Directory>
    
    # Redirect HTTP to HTTPS
    RewriteEngine On
    RewriteCond %{HTTPS} off
    RewriteRule ^(.*)$ https://%{HTTP_HOST}%{REQUEST_URI} [L,R=301]
</VirtualHost>

<VirtualHost *:443>
    ServerName mrpxtech.com
    ServerAlias www.mrpxtech.com
    DocumentRoot /var/www/mrpxtech.com/_site
    
    SSLEngine on
    SSLCertificateFile /path/to/certificate.crt
    SSLCertificateKeyFile /path/to/private.key
    
    <Directory /var/www/mrpxtech.com/_site>
        AllowOverride All
        Require all granted
    </Directory>
</VirtualHost>
```

#### Nginx Configuration
```nginx
server {
    listen 80;
    server_name mrpxtech.com www.mrpxtech.com;
    return 301 https://$server_name$request_uri;
}

server {
    listen 443 ssl http2;
    server_name mrpxtech.com www.mrpxtech.com;
    
    ssl_certificate /path/to/certificate.crt;
    ssl_certificate_key /path/to/private.key;
    
    root /var/www/mrpxtech.com/_site;
    index index.html;
    
    location / {
        try_files $uri $uri/ =404;
    }
    
    # Security headers
    add_header X-Frame-Options "SAMEORIGIN" always;
    add_header X-XSS-Protection "1; mode=block" always;
    add_header X-Content-Type-Options "nosniff" always;
    add_header Referrer-Policy "no-referrer-when-downgrade" always;
    add_header Content-Security-Policy "default-src 'self' http: https: data: blob: 'unsafe-inline'" always;
}
```

### Option 2: Azure App Service

#### Backend API Deployment
```bash
# Deploy API to Azure App Service
az webapp up --name surgicalcontrol-admin --resource-group mrpxtech-rg --runtime "NODE|18-lts"

# Configure environment variables
az webapp config appsettings set --name surgicalcontrol-admin --resource-group mrpxtech-rg --settings \
    AZURE_CLIENT_ID="your_client_id" \
    AZURE_CLIENT_SECRET="your_client_secret" \
    AZURE_TENANT_ID="your_tenant_id" \
    AZURE_SUBSCRIPTION_ID="your_subscription_id" \
    ALLOWED_ORIGIN="https://mrpxtech.com"

# Configure custom domain
az webapp config hostname add --webapp-name surgicalcontrol-admin --resource-group mrpxtech-rg --hostname api.mrpxtech.com

# Enable HTTPS
az webapp config ssl bind --certificate-thumbprint YOUR_CERT_THUMBPRINT --ssl-type SNI --name surgicalcontrol-admin --resource-group mrpxtech-rg
```

### Option 3: Cloudflare Pages

#### GitHub Actions for Cloudflare Pages
```yaml
name: Deploy to Cloudflare Pages

on:
  push:
    branches: [main]

jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      
      - name: Setup Ruby
        uses: ruby/setup-ruby@v1
        with:
          ruby-version: '3.2'
          bundler-cache: true
      
      - name: Build with Jekyll
        run: bundle exec jekyll build
        env:
          JEKYLL_ENV: production
      
      - name: Deploy to Cloudflare Pages
        uses: cloudflare/pages-action@v1
        with:
          apiToken: ${{ secrets.CLOUDFLARE_API_TOKEN }}
          accountId: ${{ secrets.CLOUDFLARE_ACCOUNT_ID }}
          projectName: mrpxtech
          directory: _site
          gitHubToken: ${{ secrets.GITHUB_TOKEN }}
```

## 🔧 Environment Configuration

### Production Environment Variables
```env
# Azure Configuration
AZURE_CLIENT_ID=your_production_client_id
AZURE_CLIENT_SECRET=your_production_client_secret
AZURE_TENANT_ID=your_tenant_id
AZURE_SUBSCRIPTION_ID=your_subscription_id

# Server Configuration
PORT=3000
ALLOWED_ORIGIN=https://mrpxtech.com
NODE_ENV=production

# Security
ADMIN_USERNAME=admin
ADMIN_PASSWORD=your_secure_password
JWT_SECRET=your_jwt_secret_key
```

### Azure Key Vault Integration
```bash
# Store secrets in Azure Key Vault
az keyvault secret set --vault-name mrpxtech-vault --name "azure-client-id" --value "your_client_id"
az keyvault secret set --vault-name mrpxtech-vault --name "azure-client-secret" --value "your_client_secret"
az keyvault secret set --vault-name mrpxtech-vault --name "admin-password" --value "your_secure_password"

# Configure App Service to use Key Vault
az webapp config appsettings set --name surgicalcontrol-admin --resource-group mrpxtech-rg --settings \
    AZURE_CLIENT_ID="@Microsoft.KeyVault(SecretUri=https://mrpxtech-vault.vault.azure.net/secrets/azure-client-id/)" \
    AZURE_CLIENT_SECRET="@Microsoft.KeyVault(SecretUri=https://mrpxtech-vault.vault.azure.net/secrets/azure-client-secret/)"
```

## 🔒 Security Configuration

### HTTPS Enforcement
```javascript
// Add to your main website
if (location.protocol !== 'https:' && location.hostname !== 'localhost') {
    location.replace(`https:${location.href.substring(location.protocol.length)}`);
}
```

### Security Headers
```apache
# Apache security headers
Header always set X-Frame-Options "SAMEORIGIN"
Header always set X-XSS-Protection "1; mode=block"
Header always set X-Content-Type-Options "nosniff"
Header always set Referrer-Policy "no-referrer-when-downgrade"
Header always set Content-Security-Policy "default-src 'self' https: data: 'unsafe-inline' 'unsafe-eval'"
```

### Rate Limiting
```javascript
// Enhanced rate limiting for production
const rateLimit = require('express-rate-limit');

const apiLimiter = rateLimit({
    windowMs: 15 * 60 * 1000, // 15 minutes
    max: 100, // limit each IP to 100 requests per windowMs
    message: 'Too many requests from this IP, please try again later.',
    standardHeaders: true,
    legacyHeaders: false,
});

const loginLimiter = rateLimit({
    windowMs: 15 * 60 * 1000, // 15 minutes
    max: 5, // limit each IP to 5 login attempts per windowMs
    message: 'Too many login attempts, please try again later.',
    standardHeaders: true,
    legacyHeaders: false,
});
```

## 📊 Monitoring and Analytics

### Azure Application Insights
```javascript
// Add to server.js
const appInsights = require('applicationinsights');
appInsights.setup('YOUR_INSTRUMENTATION_KEY').start();

// Track custom events
appInsights.defaultClient.trackEvent({
    name: 'AdminLogin',
    properties: { userId: 'admin', timestamp: new Date().toISOString() }
});
```

### Logging Configuration
```javascript
// Production logging
const winston = require('winston');

const logger = winston.createLogger({
    level: 'info',
    format: winston.format.combine(
        winston.format.timestamp(),
        winston.format.errors({ stack: true }),
        winston.format.json()
    ),
    defaultMeta: { service: 'surgicalcontrol-admin' },
    transports: [
        new winston.transports.File({ filename: 'error.log', level: 'error' }),
        new winston.transports.File({ filename: 'combined.log' })
    ]
});

if (process.env.NODE_ENV !== 'production') {
    logger.add(new winston.transports.Console({
        format: winston.format.simple()
    }));
}
```

## 🔄 CI/CD Pipeline

### GitHub Actions for Full Deployment
```yaml
name: Deploy to Production

on:
  push:
    branches: [main]

jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - name: Setup Node.js
        uses: actions/setup-node@v3
        with:
          node-version: '18'
      - name: Install dependencies
        run: npm ci
      - name: Run tests
        run: npm test

  deploy-website:
    needs: test
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - name: Setup Ruby
        uses: ruby/setup-ruby@v1
        with:
          ruby-version: '3.2'
          bundler-cache: true
      - name: Build and deploy website
        run: |
          bundle exec jekyll build
          # Deploy to your server or CDN

  deploy-api:
    needs: test
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - name: Deploy to Azure App Service
        uses: azure/webapps-deploy@v2
        with:
          app-name: 'surgicalcontrol-admin'
          publish-profile: ${{ secrets.AZURE_WEBAPP_PUBLISH_PROFILE }}
```

## 📞 Support and Maintenance

### Backup Strategy
```bash
# Automated backups
#!/bin/bash
DATE=$(date +%Y%m%d_%H%M%S)
tar -czf "backup_$DATE.tar.gz" /var/www/mrpxtech.com/
aws s3 cp "backup_$DATE.tar.gz" s3://mrpxtech-backups/
```

### Health Checks
```javascript
// Health check endpoint
app.get('/health', (req, res) => {
    res.json({
        status: 'healthy',
        timestamp: new Date().toISOString(),
        version: process.env.npm_package_version,
        uptime: process.uptime()
    });
});
```

---

**Your website is now configured for production deployment at mrpxtech.com!** 🎉 