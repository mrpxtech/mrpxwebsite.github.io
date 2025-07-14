# Azure Client Secret Creation Guide

## 🔍 Finding Certificates & Secrets

### Method 1: Through App Registration (Recommended)
1. Go to **Azure Portal**: https://portal.azure.com
2. Navigate to **Azure Active Directory**
3. Click on **App registrations**
4. Find and click on your app registration (e.g., "SurgicalControl-Admin")
5. In the left sidebar, look for **"Certificates & secrets"**
6. If you don't see it, click on **"Manage"** → **"Certificates & secrets"**

### Method 2: Direct URL
You can access it directly by replacing `YOUR_APP_ID` with your actual app ID:
```
https://portal.azure.com/#blade/Microsoft_AAD_RegisteredApps/ApplicationMenuBlade/Credentials/appId/YOUR_APP_ID
```

### Method 3: Search in Portal
1. In Azure Portal search bar, type: **"App registrations"**
2. Click on your app registration
3. Look for **"Certificates & secrets"** in the left menu

## 🚨 If "Certificates & secrets" is Missing

### Check App Registration Type
- **Single tenant**: Should have "Certificates & secrets"
- **Multi-tenant**: May have different options
- **Personal Microsoft account**: Limited functionality

### Alternative: Create New App Registration
If you can't find the section, create a new app registration:

```bash
# Using Azure CLI
az ad sp create-for-rbac --name "SurgicalControl-Admin-New" --role contributor --scopes /subscriptions/YOUR_SUBSCRIPTION_ID
```

## 🔑 Creating Client Secret

### Step 1: Navigate to Certificates & Secrets
1. In your app registration, click **"Certificates & secrets"**
2. You should see two tabs: **"Client secrets"** and **"Certificates"**

### Step 2: Create New Client Secret
1. Click on the **"Client secrets"** tab
2. Click **"+ New client secret"**
3. Add a description (e.g., "SurgicalControl Admin API")
4. Choose expiration:
   - **Recommended**: 12 months for development
   - **Production**: 24 months maximum
5. Click **"Add"**

### Step 3: Copy the Secret Value
⚠️ **IMPORTANT**: Copy the secret value immediately! You won't be able to see it again.

## 🔧 Alternative: Using Azure CLI

If the portal method doesn't work, use Azure CLI:

```bash
# Login to Azure
az login

# Create app registration with service principal
az ad sp create-for-rbac --name "SurgicalControl-Admin" --role contributor --scopes /subscriptions/YOUR_SUBSCRIPTION_ID

# The output will include:
# {
#   "appId": "YOUR_CLIENT_ID",
#   "displayName": "SurgicalControl-Admin",
#   "password": "YOUR_CLIENT_SECRET",  ← This is your secret!
#   "tenant": "YOUR_TENANT_ID"
# }
```

## 📋 Required Information

Once you have the client secret, you'll need these values:

```env
AZURE_CLIENT_ID=your_app_id_here
AZURE_CLIENT_SECRET=your_client_secret_here
AZURE_TENANT_ID=your_tenant_id_here
AZURE_SUBSCRIPTION_ID=your_subscription_id_here
```

## 🔍 Finding Your Values

### Client ID (App ID)
- Found in app registration overview
- Also called "Application (client) ID"

### Tenant ID
- Found in app registration overview
- Also called "Directory (tenant) ID"

### Subscription ID
```bash
# Get subscription ID
az account show --query id -o tsv
```

## 🛡️ Security Best Practices

### 1. Store Secrets Securely
- Never commit secrets to git
- Use Azure Key Vault for production
- Use environment variables locally

### 2. Rotate Secrets Regularly
- Set expiration dates
- Rotate every 90 days for production
- Keep track of secret expiration

### 3. Use Least Privilege
- Only grant necessary permissions
- Use specific resource group scopes
- Review permissions regularly

## 🔧 Troubleshooting

### "Certificates & secrets not visible"
- Check if you have proper permissions
- Try creating a new app registration
- Use Azure CLI as alternative

### "Secret not working"
- Verify secret hasn't expired
- Check if secret was copied correctly
- Ensure app registration is active

### "Permission denied"
- Check role assignments
- Verify subscription access
- Ensure app registration is in correct tenant

## 📞 Support

If you still can't access Certificates & secrets:

1. **Check your Azure role**: You need "Application Administrator" or "Global Administrator"
2. **Try different browser**: Clear cache and cookies
3. **Use Azure CLI**: More reliable for automation
4. **Contact Azure support**: If portal issues persist

## 🎯 Quick Setup Commands

```bash
# Complete setup with Azure CLI
az login
az ad sp create-for-rbac --name "SurgicalControl-Admin" --role contributor --scopes /subscriptions/$(az account show --query id -o tsv)

# Save the output and update your .env file
```

---

**Once you have your client secret, update your `.env` file and run the setup script!** 