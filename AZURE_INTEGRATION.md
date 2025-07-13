# Azure Integration Guide

## 🎯 Overview

This guide will help you connect your hidden admin dashboard to your actual Azure VM usage. The integration provides real-time monitoring and management of your Azure VMs.

## 📋 Prerequisites

- ✅ Azure subscription with VMs
- ✅ Azure CLI installed
- ✅ Node.js installed
- ✅ Git repository set up

## 🚀 Quick Setup

### Step 1: Run Setup Script
```bash
./setup-azure.sh
```

This script will:
- Check prerequisites
- Install Node.js dependencies
- Create environment file
- Verify Azure login
- Set subscription ID

### Step 2: Create Azure Service Principal

#### Option A: Using Azure CLI (Recommended)
```bash
# Login to Azure
az login

# Create service principal
az ad sp create-for-rbac --name "SurgicalControl-Admin" --role contributor --scopes /subscriptions/YOUR_SUBSCRIPTION_ID

# Save the output - you'll need these values:
# {
#   "appId": "YOUR_CLIENT_ID",
#   "displayName": "SurgicalControl-Admin",
#   "password": "YOUR_CLIENT_SECRET",
#   "tenant": "YOUR_TENANT_ID"
# }
```

#### Option B: Using Azure Portal
1. Go to Azure Portal → Azure Active Directory → App registrations
2. Click "New registration"
3. Name: "SurgicalControl-Admin"
4. Select "Accounts in this organizational directory only"
5. Click "Register"
6. Note the Application (client) ID and Directory (tenant) ID
7. Go to "Certificates & secrets" → "New client secret"
8. Add description and expiration, then copy the secret value

### Step 3: Configure Environment

Edit the `.env` file with your Azure credentials:
```env
AZURE_CLIENT_ID=your_client_id_here
AZURE_CLIENT_SECRET=your_client_secret_here
AZURE_TENANT_ID=your_tenant_id_here
AZURE_SUBSCRIPTION_ID=your_subscription_id_here
PORT=3000
ALLOWED_ORIGIN=http://localhost:8000
```

### Step 4: Start the Backend API
```bash
npm start
```

### Step 5: Access Admin Dashboard
1. **Local Development**: Start your website: `python3 -m http.server 8000`
2. **Local Access**: Go to: `http://localhost:8000/admin-login.html`
3. **Production Access**: Go to: `https://mrpxtech.com/admin-login.html`
4. Login with: `admin` / `surgical2024`
5. View your real Azure VM data!

## 🔧 API Endpoints

The backend provides these endpoints:

- `GET /api/vms` - Get all VMs with metrics
- `GET /api/vms/summary` - Get summary statistics
- `POST /api/vms/:resourceGroup/:vmName/start` - Start VM
- `POST /api/vms/:resourceGroup/:vmName/stop` - Stop VM
- `POST /api/vms/:resourceGroup/:vmName/restart` - Restart VM
- `GET /api/health` - Health check

## 📊 Features

### Real-time Monitoring
- ✅ VM status (running, stopped, starting, etc.)
- ✅ CPU usage percentage
- ✅ Memory usage percentage
- ✅ Disk I/O metrics
- ✅ Network I/O metrics
- ✅ Uptime tracking
- ✅ Region and size information

### VM Management
- ✅ Start VMs
- ✅ Stop VMs
- ✅ Restart VMs
- ✅ Real-time status updates

### Analytics
- ✅ Total VM count
- ✅ Running vs stopped VMs
- ✅ Average resource usage
- ✅ Estimated monthly cost

## 🔒 Security Considerations

### Production Deployment
1. **Use Azure Key Vault** for storing secrets
2. **Implement proper authentication** (JWT tokens)
3. **Enable HTTPS** for all communications
4. **Add rate limiting** and IP restrictions
5. **Use managed identities** instead of service principals
6. **Implement audit logging**

### Service Principal Permissions
- **Reader role**: For monitoring only
- **Virtual Machine Contributor**: For VM management
- **Monitor Reader**: For metrics access

## 🚀 Production Deployment

### Option 1: Azure App Service
```bash
# Deploy to Azure App Service
az webapp up --name surgicalcontrol-admin --resource-group your-rg --runtime "NODE|18-lts"

# Configure custom domain
az webapp config hostname add --webapp-name surgicalcontrol-admin --resource-group your-rg --hostname api.mrpxtech.com
```

### Option 2: Azure Container Instances
```bash
# Build and deploy Docker container
docker build -t surgicalcontrol-admin .
az container create --resource-group your-rg --name admin-api --image surgicalcontrol-admin
```

### Option 3: Azure Functions
Convert the API to serverless functions for cost optimization.

## 🔍 Troubleshooting

### Common Issues

#### "Authentication failed"
- Verify service principal credentials in `.env`
- Check if service principal has correct permissions
- Ensure subscription ID is correct

#### "No VMs found"
- Verify you have VMs in your subscription
- Check if service principal has Reader access
- Ensure VMs are in accessible resource groups

#### "API connection failed"
- Check if backend server is running (`npm start`)
- Verify CORS settings in `server.js`
- Check firewall/network settings

#### "Metrics not showing"
- Ensure VMs have monitoring enabled
- Check if Azure Monitor is configured
- Verify service principal has Monitor Reader role

### Debug Mode
Enable debug logging by setting:
```env
DEBUG=azure:*
```

## 📈 Monitoring and Alerts

### Azure Monitor Integration
- Set up alerts for high CPU/memory usage
- Create dashboards for VM performance
- Configure automated scaling rules

### Cost Optimization
- Monitor VM usage patterns
- Set up auto-shutdown for dev/test VMs
- Use Azure Reserved Instances for production

## 🔄 Updates and Maintenance

### Regular Tasks
1. **Rotate service principal secrets** every 90 days
2. **Update Node.js dependencies** monthly
3. **Review Azure permissions** quarterly
4. **Monitor API usage** and performance

### Backup Strategy
- Backup `.env` configuration
- Document customizations
- Version control all changes

## 📞 Support

For issues with:
- **Azure setup**: Check `azure-setup.md`
- **API problems**: Review server logs
- **Dashboard issues**: Check browser console
- **Deployment**: See `DEPLOYMENT.md`

---

**Your admin dashboard is now connected to real Azure VM data!** 🎉 