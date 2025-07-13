#!/bin/bash

echo "🚀 Setting up Azure Integration for SurgicalControl Admin Dashboard"
echo "================================================================"

# Check if Node.js is installed
if ! command -v node &> /dev/null; then
    echo "❌ Node.js is not installed. Please install Node.js first."
    echo "   Visit: https://nodejs.org/"
    exit 1
fi

# Check if Azure CLI is installed
if ! command -v az &> /dev/null; then
    echo "❌ Azure CLI is not installed. Please install Azure CLI first."
    echo "   Visit: https://docs.microsoft.com/en-us/cli/azure/install-azure-cli"
    exit 1
fi

echo "✅ Prerequisites check passed"

# Install dependencies
echo "📦 Installing Node.js dependencies..."
npm install

# Check if .env file exists
if [ ! -f .env ]; then
    echo "📝 Creating .env file from template..."
    cp env.example .env
    echo "⚠️  Please edit .env file with your Azure credentials"
    echo "   You can get these from the Azure Portal or Azure CLI"
else
    echo "✅ .env file already exists"
fi

# Check Azure login
echo "🔐 Checking Azure login status..."
if ! az account show &> /dev/null; then
    echo "⚠️  Not logged into Azure. Please run: az login"
    echo "   Then run this script again"
    exit 1
fi

echo "✅ Azure login verified"

# Get subscription info
SUBSCRIPTION_ID=$(az account show --query id -o tsv)
echo "📋 Current subscription: $SUBSCRIPTION_ID"

# Update .env with subscription ID
sed -i "s/your_subscription_id_here/$SUBSCRIPTION_ID/" .env

echo ""
echo "🎉 Setup complete!"
echo ""
echo "Next steps:"
echo "1. Edit .env file with your Azure service principal credentials"
echo "2. Run: npm start"
echo "3. Access admin dashboard at: http://localhost:8000/admin-login.html"
echo ""
echo "For help creating service principal, see: azure-setup.md" 