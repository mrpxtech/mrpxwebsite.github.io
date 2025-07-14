#!/bin/bash

echo "🔐 Azure Credentials Setup for SurgicalControl"
echo "=============================================="

# Check if Azure CLI is installed
if ! command -v az &> /dev/null; then
    echo "❌ Azure CLI is not installed."
    echo "   Please install it first: https://docs.microsoft.com/en-us/cli/azure/install-azure-cli"
    exit 1
fi

echo "✅ Azure CLI found"

# Check if logged in
if ! az account show &> /dev/null; then
    echo "🔑 Please log in to Azure..."
    az login
fi

echo "✅ Azure login verified"

# Get subscription info
SUBSCRIPTION_ID=$(az account show --query id -o tsv)
SUBSCRIPTION_NAME=$(az account show --query name -o tsv)
TENANT_ID=$(az account show --query tenantId -o tsv)

echo "📋 Current subscription: $SUBSCRIPTION_NAME ($SUBSCRIPTION_ID)"
echo "🏢 Tenant ID: $TENANT_ID"

# Ask user if they want to create new app registration
echo ""
read -p "Do you want to create a new app registration? (y/n): " -n 1 -r
echo ""

if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "🚀 Creating new app registration..."
    
    # Create app registration with service principal
    echo "Creating service principal..."
    SP_OUTPUT=$(az ad sp create-for-rbac --name "SurgicalControl-Admin" --role contributor --scopes "/subscriptions/$SUBSCRIPTION_ID" --output json)
    
    if [ $? -eq 0 ]; then
        echo "✅ App registration created successfully!"
        
        # Parse the output
        CLIENT_ID=$(echo $SP_OUTPUT | jq -r '.appId')
        CLIENT_SECRET=$(echo $SP_OUTPUT | jq -r '.password')
        
        echo ""
        echo "🔑 Your Azure credentials:"
        echo "=========================="
        echo "AZURE_CLIENT_ID=$CLIENT_ID"
        echo "AZURE_CLIENT_SECRET=$CLIENT_SECRET"
        echo "AZURE_TENANT_ID=$TENANT_ID"
        echo "AZURE_SUBSCRIPTION_ID=$SUBSCRIPTION_ID"
        echo ""
        
        # Ask if user wants to update .env file
        read -p "Do you want to update your .env file? (y/n): " -n 1 -r
        echo ""
        
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            if [ -f .env ]; then
                # Backup existing .env
                cp .env .env.backup
                echo "📁 Backed up existing .env to .env.backup"
            fi
            
            # Create/update .env file
            cat > .env << EOF
# Azure Configuration
AZURE_CLIENT_ID=$CLIENT_ID
AZURE_CLIENT_SECRET=$CLIENT_SECRET
AZURE_TENANT_ID=$TENANT_ID
AZURE_SUBSCRIPTION_ID=$SUBSCRIPTION_ID

# Server Configuration
PORT=3000
ALLOWED_ORIGIN=https://mrpxtech.com

# Security (change these in production)
ADMIN_USERNAME=admin
ADMIN_PASSWORD=surgical2024
EOF
            
            echo "✅ .env file updated!"
            echo "🔒 Remember to keep your .env file secure and never commit it to git!"
        fi
        
    else
        echo "❌ Failed to create app registration"
        echo "   Please check your Azure permissions"
        exit 1
    fi
    
else
    echo "📝 Manual setup mode"
    echo ""
    echo "Please provide your existing app registration details:"
    echo ""
    
    read -p "Enter your Client ID (App ID): " CLIENT_ID
    read -p "Enter your Client Secret: " CLIENT_SECRET
    
    echo ""
    echo "🔑 Your Azure credentials:"
    echo "=========================="
    echo "AZURE_CLIENT_ID=$CLIENT_ID"
    echo "AZURE_CLIENT_SECRET=$CLIENT_SECRET"
    echo "AZURE_TENANT_ID=$TENANT_ID"
    echo "AZURE_SUBSCRIPTION_ID=$SUBSCRIPTION_ID"
    echo ""
    
    # Ask if user wants to update .env file
    read -p "Do you want to update your .env file? (y/n): " -n 1 -r
    echo ""
    
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        if [ -f .env ]; then
            # Backup existing .env
            cp .env .env.backup
            echo "📁 Backed up existing .env to .env.backup"
        fi
        
        # Create/update .env file
        cat > .env << EOF
# Azure Configuration
AZURE_CLIENT_ID=$CLIENT_ID
AZURE_CLIENT_SECRET=$CLIENT_SECRET
AZURE_TENANT_ID=$TENANT_ID
AZURE_SUBSCRIPTION_ID=$SUBSCRIPTION_ID

# Server Configuration
PORT=3000
ALLOWED_ORIGIN=https://mrpxtech.com

# Security (change these in production)
ADMIN_USERNAME=admin
ADMIN_PASSWORD=surgical2024
EOF
        
        echo "✅ .env file updated!"
        echo "🔒 Remember to keep your .env file secure and never commit it to git!"
    fi
fi

echo ""
echo "🎉 Setup complete!"
echo ""
echo "Next steps:"
echo "1. Install Node.js dependencies: npm install"
echo "2. Start the API server: npm start"
echo "3. Access admin dashboard: http://localhost:8000/admin-login.html"
echo ""
echo "For troubleshooting, see: azure-client-secret-guide.md" 