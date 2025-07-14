# Azure Subscription Information

## 📋 Your Azure Details

- **Subscription Name**: Azure subscription 1
- **Subscription ID**: `3e592de9-7e72-4d0a-aa13-2f3cb776c014`
- **Tenant ID**: `7b2f8db1-0b67-4907-b54f-52dd9f54eaed`

## 🔧 Quick Commands

### Get Subscription Info
```bash
# Get current subscription
az account show --query id -o tsv

# Get all subscriptions
az account list --query "[].{name:name, id:id, isDefault:isDefault}" -o table

# Get tenant ID
az account show --query tenantId -o tsv
```

### Switch Subscriptions (if you have multiple)
```bash
# List all subscriptions
az account list --query "[].{name:name, id:id}" -o table

# Switch to a specific subscription
az account set --subscription "SUBSCRIPTION_NAME_OR_ID"
```

## 📝 Environment Variables

Your `.env` file should contain:
```env
AZURE_SUBSCRIPTION_ID=3e592de9-7e72-4d0a-aa13-2f3cb776c014
AZURE_TENANT_ID=7b2f8db1-0b67-4907-b54f-52dd9f54eaed
AZURE_CLIENT_ID=your_client_id_here
AZURE_CLIENT_SECRET=your_client_secret_here
```

## 🚀 Next Steps

1. **Get Client ID and Secret**: Run `./get-azure-credentials.sh`
2. **Update .env file**: Add your CLIENT_ID and CLIENT_SECRET
3. **Start the API**: Run `npm start`
4. **Access Dashboard**: Go to `http://localhost:8000/admin-login.html`

## 🔍 Finding Subscription ID in Azure Portal

1. Go to [Azure Portal](https://portal.azure.com)
2. Click **"Subscriptions"** in the left sidebar
3. Your subscription ID will be listed in the table
4. Click on your subscription name for more details

## 🆘 Troubleshooting

### "No subscription found"
- Make sure you're logged in: `az login`
- Check if you have access to the subscription
- Verify you're in the correct tenant

### "Permission denied"
- Ensure you have proper permissions on the subscription
- Contact your Azure administrator if needed

---

**Keep this information secure and never commit it to version control!** 