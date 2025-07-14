#!/bin/bash

echo "🔐 Azure Environment Setup"
echo "=========================="

# Get Azure subscription info
SUBSCRIPTION_ID=$(az account show --query id -o tsv)
TENANT_ID=$(az account show --query tenantId -o tsv)
SUBSCRIPTION_NAME=$(az account show --query name -o tsv)

echo "✅ Found Azure subscription:"
echo "   Name: $SUBSCRIPTION_NAME"
echo "   ID: $SUBSCRIPTION_ID"
echo "   Tenant ID: $TENANT_ID"
echo ""

# Create .env file
cat > .env << EOF
# Azure Configuration
AZURE_CLIENT_ID=your_client_id_here
AZURE_CLIENT_SECRET=your_client_secret_here
AZURE_TENANT_ID=$TENANT_ID
AZURE_SUBSCRIPTION_ID=$SUBSCRIPTION_ID

# Server Configuration
PORT=3000
ALLOWED_ORIGIN=https://mrpxtech.com

# Security (change these in production)
ADMIN_USERNAME=admin
ADMIN_PASSWORD=surgical2024
EOF

echo "✅ Created .env file with your subscription details"
echo ""
echo "📝 Next steps:"
echo "1. Edit .env file and add your CLIENT_ID and CLIENT_SECRET"
echo "2. Run: npm start"
echo "3. Access admin dashboard: http://localhost:8000/admin-login.html"
echo ""
echo "🔑 To get CLIENT_ID and CLIENT_SECRET, run:"
echo "   ./get-azure-credentials.sh" 