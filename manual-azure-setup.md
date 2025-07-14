# Manual Azure Setup Guide

## 🚨 If You Can't Find "Certificates & Secrets"

This is a common issue! Here are several solutions:

## 🔍 **Solution 1: Check App Registration Type**

### Step 1: Verify App Registration
1. Go to **Azure Portal**: https://portal.azure.com
2. Search for **"App registrations"**
3. Click on your app registration
4. Check the **"Supported account types"** field

### Step 2: App Registration Types
- **"Accounts in this organizational directory only"** → Should have Certificates & secrets
- **"Accounts in any organizational directory"** → Should have Certificates & secrets  
- **"Accounts in any organizational directory and personal Microsoft accounts"** → May have limited options
- **"Personal Microsoft accounts only"** → Limited functionality

## 🔧 **Solution 2: Create New App Registration**

If your current app registration doesn't show Certificates & secrets, create a new one:

### Step 1: Create New Registration
1. Go to **Azure Portal** → **Azure Active Directory** → **App registrations**
2. Click **"New registration"**
3. **Name**: "SurgicalControl-Admin"
4. **Supported account types**: Select **"Accounts in this organizational directory only"**
5. Click **"Register"**

### Step 2: Find Certificates & Secrets
1. In your new app registration, look for **"Certificates & secrets"** in the left menu
2. If you see it, click on it
3. Click **"Client secrets"** tab
4. Click **"+ New client secret"**

### Step 3: Create Client Secret
1. **Description**: "SurgicalControl Admin API"
2. **Expires**: Choose 12 months (recommended)
3. Click **"Add"**
4. **IMPORTANT**: Copy the secret value immediately!

## 🔑 **Solution 3: Manual Credential Collection**

If you still can't access Certificates & secrets, collect your credentials manually:

### Step 1: Get Client ID
1. In your app registration overview
2. Copy the **"Application (client) ID"**

### Step 2: Get Tenant ID
1. In your app registration overview
2. Copy the **"Directory (tenant) ID"**

### Step 3: Get Subscription ID
1. Go to **Azure Portal** → **Subscriptions**
2. Copy your subscription ID

### Step 4: Create Client Secret (Alternative Method)
If you can't create a client secret in the portal, you can:

#### Option A: Use Azure CLI (if available)
```bash
# Install Azure CLI first
curl -sL https://aka.ms/InstallAzureCLIDeb | sudo bash

# Login and create service principal
az login
az ad sp create-for-rbac --name "SurgicalControl-Admin" --role contributor --scopes /subscriptions/YOUR_SUBSCRIPTION_ID
```

#### Option B: Use PowerShell
```powershell
# Install Azure PowerShell module
Install-Module -Name Az -AllowClobber

# Login and create service principal
Connect-AzAccount
New-AzADServicePrincipal -DisplayName "SurgicalControl-Admin" -Role Contributor -Scope "/subscriptions/YOUR_SUBSCRIPTION_ID"
```

## 📋 **Manual Configuration**

Once you have your credentials, create your `.env` file manually:

```env
# Azure Configuration
AZURE_CLIENT_ID=your_client_id_here
AZURE_CLIENT_SECRET=your_client_secret_here
AZURE_TENANT_ID=your_tenant_id_here
AZURE_SUBSCRIPTION_ID=your_subscription_id_here

# Server Configuration
PORT=3000
ALLOWED_ORIGIN=https://mrpxtech.com

# Security (change these in production)
ADMIN_USERNAME=admin
ADMIN_PASSWORD=surgical2024
```

## 🔍 **Finding Your Values**

### Where to Find Each Value:

1. **Client ID**: App registration → Overview → Application (client) ID
2. **Tenant ID**: App registration → Overview → Directory (tenant) ID  
3. **Subscription ID**: Azure Portal → Subscriptions → Your subscription
4. **Client Secret**: App registration → Certificates & secrets → Client secrets

## 🛠️ **Installation Help**

### Install Azure CLI (Ubuntu/Debian)
```bash
# Install Azure CLI
curl -sL https://aka.ms/InstallAzureCLIDeb | sudo bash

# Install jq for JSON parsing
sudo apt-get install jq

# Login to Azure
az login
```

### Install Azure CLI (Windows)
```powershell
# Install via winget
winget install Microsoft.AzureCLI

# Or download from: https://docs.microsoft.com/en-us/cli/azure/install-azure-cli-windows
```

### Install Azure CLI (macOS)
```bash
# Install via Homebrew
brew install azure-cli

# Login to Azure
az login
```

## 🔧 **Alternative: Use Existing App Registration**

If you have an existing app registration that works:

1. **Find your existing app registration** in Azure Portal
2. **Check if it has a client secret** in Certificates & secrets
3. **If it has a secret**, use that one
4. **If it doesn't have a secret**, create a new one in the same app registration

## 🆘 **Still Having Issues?**

### Common Problems and Solutions:

1. **"Certificates & secrets not visible"**
   - Check your Azure role (need Application Administrator or Global Administrator)
   - Try creating a new app registration
   - Use a different browser or clear cache

2. **"Permission denied"**
   - Ensure you're in the correct Azure tenant
   - Check if you have proper permissions
   - Contact your Azure administrator

3. **"App registration not found"**
   - Check if you're in the correct directory
   - Search for the app registration by name
   - Create a new one if needed

## 📞 **Get Help**

If you're still stuck:

1. **Check the detailed guide**: `azure-client-secret-guide.md`
2. **Use the automated script**: `./get-azure-credentials.sh` (after installing Azure CLI)
3. **Contact Azure support**: If portal issues persist
4. **Use Azure CLI**: More reliable than portal for automation

---

**Once you have your credentials, you can proceed with the Azure integration!** 