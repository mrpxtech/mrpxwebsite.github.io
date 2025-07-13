# Admin Dashboard Access

## Overview
The admin dashboard provides real-time monitoring and management of Azure VM usage. It's hidden from the main navigation but accessible via direct URL.

## Access Instructions

### 1. View the Website
- **Local Development**: The website is running on `http://localhost:8000`
- **Production**: Deploy to GitHub Pages or your preferred hosting service

### 2. Access Admin Dashboard

#### Method 1: Direct URL Access
1. Navigate to: `http://localhost:8000/admin-login.html`
2. Use the following credentials:
   - **Username**: `admin`
   - **Password**: `surgical2024`

#### Method 2: Hidden Link
Add this to your browser's address bar after the main site URL:
```
/admin-login.html
```

## Admin Dashboard Features

### VM Monitoring
- Real-time status of all Azure VMs
- CPU, Memory, Disk, and Network usage
- VM region and size information
- Uptime tracking

### VM Management
- Start/Stop VMs
- Restart VMs
- Real-time status updates

### Usage Analytics
- Total VM count
- Running vs Stopped VMs
- Average resource usage
- Monthly cost estimation

### Security Features
- Session-based authentication (24-hour validity)
- Automatic logout on session expiry
- Secure login page

## Production Considerations

### Security Improvements Needed
1. **Server-side Authentication**: Replace client-side auth with proper JWT tokens
2. **HTTPS**: Ensure all admin pages use HTTPS
3. **Rate Limiting**: Implement login attempt limits
4. **Audit Logging**: Log all admin actions
5. **Multi-factor Authentication**: Add 2FA for admin access

### Azure Integration
1. **Azure SDK**: Replace mock data with real Azure API calls
2. **Service Principal**: Use Azure service principal for API access
3. **Resource Groups**: Scope access to specific resource groups
4. **Monitoring**: Integrate with Azure Monitor for real metrics

### Code Example for Azure Integration
```javascript
// Example Azure SDK integration
const { ComputeManagementClient } = require('@azure/arm-compute');
const { DefaultAzureCredential } = require('@azure/identity');

const credential = new DefaultAzureCredential();
const client = new ComputeManagementClient(credential, subscriptionId);

async function getVMs() {
    const vms = [];
    for await (const vm of client.virtualMachines.list(resourceGroupName)) {
        vms.push({
            id: vm.id,
            name: vm.name,
            status: vm.instanceView?.statuses?.[1]?.displayStatus || 'Unknown',
            // ... other properties
        });
    }
    return vms;
}
```

## File Structure
```
├── index.html              # Main website
├── admin-login.html        # Admin login page
├── admin.html              # Admin dashboard
├── styles.css              # Shared styles
├── script.js               # Main website scripts
└── ADMIN_README.md         # This file
```

## Troubleshooting

### Can't Access Admin Page
1. Check if the server is running on port 8000
2. Verify the URL: `http://localhost:8000/admin-login.html`
3. Clear browser cache and cookies
4. Try incognito/private browsing mode

### Login Issues
1. Verify credentials: `admin` / `surgical2024`
2. Check browser console for JavaScript errors
3. Ensure JavaScript is enabled
4. Clear localStorage and try again

### Dashboard Not Loading
1. Check browser console for errors
2. Verify all files are in the same directory
3. Ensure internet connection for external resources
4. Try refreshing the page

## Support
For technical issues or security concerns, contact your system administrator. 