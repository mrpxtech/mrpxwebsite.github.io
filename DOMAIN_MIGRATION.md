# Domain Migration Guide: GitHub Pages → mrpxtech.com

## 🎯 Migration Summary

Your website has been successfully configured to work with the new domain `mrpxtech.com` instead of the previous GitHub Pages URL.

## ✅ Changes Made

### 1. Configuration Updates
- ✅ **Jekyll Config**: Updated `_config.yml` with new domain
- ✅ **Admin Dashboard**: Updated API endpoints for production
- ✅ **Environment Variables**: Updated CORS settings for new domain
- ✅ **GitHub Actions**: Updated deployment workflow for custom hosting

### 2. New Files Created
- ✅ `production-config.md` - Complete production setup guide
- ✅ `DOMAIN_MIGRATION.md` - This migration guide

### 3. Updated Files
- ✅ `_config.yml` - Domain changed to `https://mrpxtech.com`
- ✅ `admin.html` - API endpoints now use `api.mrpxtech.com`
- ✅ `env.example` - CORS origin updated
- ✅ `DEPLOYMENT.md` - Updated deployment instructions
- ✅ `AZURE_INTEGRATION.md` - Updated access URLs
- ✅ `.github/workflows/deploy.yml` - New deployment workflow

## 🌐 New URLs

### Website Access
- **Main Site**: `https://mrpxtech.com`
- **Admin Login**: `https://mrpxtech.com/admin-login.html`
- **Admin Dashboard**: `https://mrpxtech.com/admin.html`

### API Endpoints
- **Local Development**: `http://localhost:3000/api`
- **Production**: `https://api.mrpxtech.com/api`

## 🚀 Deployment Options

### Option 1: Traditional Web Server
```bash
# Build the site
bundle exec jekyll build

# Upload to your server
scp -r _site/* user@your-server:/var/www/mrpxtech.com/
```

### Option 2: Azure App Service
```bash
# Deploy website to Azure Storage or App Service
az storage blob upload-batch --account-name your-storage --source _site --destination '$web'

# Deploy API to App Service
az webapp up --name surgicalcontrol-admin --resource-group your-rg
```

### Option 3: Cloudflare Pages
- Connect your GitHub repository to Cloudflare Pages
- Set build command: `bundle exec jekyll build`
- Set build output directory: `_site`

## 🔧 Required Setup

### 1. DNS Configuration
Configure your DNS provider with these records:
```
A     @           YOUR_SERVER_IP
CNAME www         mrpxtech.com
CNAME api         surgicalcontrol-admin.azurewebsites.net
```

### 2. SSL Certificate
- **Let's Encrypt**: Free SSL certificates
- **Cloudflare**: Free SSL with CDN
- **Azure**: Managed certificates for App Service

### 3. Environment Variables
Update your production environment:
```env
ALLOWED_ORIGIN=https://mrpxtech.com
NODE_ENV=production
```

## 🔒 Security Updates

### HTTPS Enforcement
The admin dashboard now enforces HTTPS in production:
```javascript
if (location.protocol !== 'https:' && location.hostname !== 'localhost') {
    location.replace(`https:${location.href.substring(location.protocol.length)}`);
}
```

### CORS Configuration
API now only accepts requests from `https://mrpxtech.com`:
```javascript
cors({
    origin: 'https://mrpxtech.com',
    credentials: true
})
```

## 📊 Monitoring

### Health Checks
- **Website**: `https://mrpxtech.com/health`
- **API**: `https://api.mrpxtech.com/health`

### Analytics
- Update Google Analytics property to new domain
- Update any tracking scripts with new URLs
- Configure Azure Application Insights for API monitoring

## 🔄 Migration Checklist

### Pre-Migration
- [ ] DNS records configured
- [ ] SSL certificate obtained
- [ ] Server/hosting environment ready
- [ ] Backup of current site

### Migration Day
- [ ] Deploy new site to production
- [ ] Test all functionality
- [ ] Verify admin dashboard access
- [ ] Check API connectivity
- [ ] Test Azure VM integration

### Post-Migration
- [ ] Update any external links
- [ ] Configure monitoring and alerts
- [ ] Set up automated backups
- [ ] Document new procedures

## 🆘 Troubleshooting

### Common Issues

#### "Site not loading"
- Check DNS propagation (can take up to 48 hours)
- Verify SSL certificate is valid
- Check server configuration

#### "Admin dashboard not working"
- Verify API server is running
- Check CORS configuration
- Ensure HTTPS is properly configured

#### "Azure integration failing"
- Verify service principal credentials
- Check API endpoint accessibility
- Ensure proper permissions are set

### Rollback Plan
If issues occur, you can quickly rollback:
1. Revert DNS changes
2. Deploy previous version
3. Update configuration files
4. Test functionality

## 📞 Support

For migration assistance:
1. Check `production-config.md` for detailed setup
2. Review `AZURE_INTEGRATION.md` for API setup
3. Use `DEPLOYMENT.md` for deployment guidance
4. Contact your hosting provider for server-specific issues

---

**Migration to mrpxtech.com is complete!** 🎉

Your website and admin dashboard are now configured for the new domain with enhanced security and production-ready deployment options. 