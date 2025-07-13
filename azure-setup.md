# Azure Integration Setup Guide

## Prerequisites
- Azure CLI installed
- Azure subscription with VMs
- Node.js (for backend API)

## Step 1: Create Azure Service Principal

### Option A: Using Azure CLI
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

### Option B: Using Azure Portal
1. Go to Azure Portal → Azure Active Directory → App registrations
2. Click "New registration"
3. Name: "SurgicalControl-Admin"
4. Select "Accounts in this organizational directory only"
5. Click "Register"
6. Note the Application (client) ID and Directory (tenant) ID
7. Go to "Certificates & secrets" → "New client secret"
8. Add description and expiration, then copy the secret value

## Step 2: Assign Permissions

```bash
# Get your subscription ID
az account show --query id -o tsv

# Assign Reader role to service principal
az role assignment create --assignee YOUR_CLIENT_ID --role "Reader" --scope /subscriptions/YOUR_SUBSCRIPTION_ID

# For VM management, also assign Virtual Machine Contributor
az role assignment create --assignee YOUR_CLIENT_ID --role "Virtual Machine Contributor" --scope /subscriptions/YOUR_SUBSCRIPTION_ID
```

## Step 3: Environment Variables

Create a `.env` file in your project root:
```env
AZURE_CLIENT_ID=your_client_id
AZURE_CLIENT_SECRET=your_client_secret
AZURE_TENANT_ID=your_tenant_id
AZURE_SUBSCRIPTION_ID=your_subscription_id
```

## Step 4: Install Azure SDK

```bash
npm init -y
npm install @azure/arm-compute @azure/arm-monitor @azure/identity dotenv
```

## Security Notes
- Never commit `.env` file to git
- Use Azure Key Vault for production secrets
- Implement proper authentication for admin access
- Use least privilege principle for service principal roles 