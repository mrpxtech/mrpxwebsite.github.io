require('dotenv').config();
const express = require('express');
const cors = require('cors');
const helmet = require('helmet');
const rateLimit = require('express-rate-limit');
const bcrypt = require('bcryptjs');
const jwt = require('jsonwebtoken');
const sqlite3 = require('sqlite3').verbose();
const { v4: uuidv4 } = require('uuid');
const { DefaultAzureCredential } = require('@azure/identity');
const { ComputeManagementClient } = require('@azure/arm-compute');
const { MonitorClient } = require('@azure/arm-monitor');
const { ConsumptionManagementClient } = require('@azure/arm-consumption');

const app = express();
const PORT = process.env.PORT || 3000;

// Security middleware
app.use(helmet());
app.use(cors({
    origin: process.env.ALLOWED_ORIGIN || 'http://localhost:8000',
    credentials: true
}));

// Rate limiting
const limiter = rateLimit({
    windowMs: 15 * 60 * 1000, // 15 minutes
    max: 100 // limit each IP to 100 requests per windowMs
});
app.use('/api/', limiter);

app.use(express.json());

// Database initialization
const db = new sqlite3.Database('./users.db');

// Create tables if they don't exist
db.serialize(() => {
    // Users table
    db.run(`CREATE TABLE IF NOT EXISTS users (
        id TEXT PRIMARY KEY,
        fullName TEXT NOT NULL,
        email TEXT UNIQUE NOT NULL,
        password TEXT NOT NULL,
        plan TEXT NOT NULL,
        balance REAL DEFAULT 0.0,
        createdAt DATETIME DEFAULT CURRENT_TIMESTAMP,
        updatedAt DATETIME DEFAULT CURRENT_TIMESTAMP
    )`);

    // Usage sessions table
    db.run(`CREATE TABLE IF NOT EXISTS usage_sessions (
        id TEXT PRIMARY KEY,
        userId TEXT NOT NULL,
        startTime DATETIME DEFAULT CURRENT_TIMESTAMP,
        endTime DATETIME,
        duration INTEGER DEFAULT 0,
        cost REAL DEFAULT 0.0,
        FOREIGN KEY (userId) REFERENCES users (id)
    )`);

    // Monthly usage tracking
    db.run(`CREATE TABLE IF NOT EXISTS monthly_usage (
        id TEXT PRIMARY KEY,
        userId TEXT NOT NULL,
        year INTEGER NOT NULL,
        month INTEGER NOT NULL,
        totalSeconds INTEGER DEFAULT 0,
        totalCost REAL DEFAULT 0.0,
        FOREIGN KEY (userId) REFERENCES users (id),
        UNIQUE(userId, year, month)
    )`);
});

// JWT Secret
const JWT_SECRET = process.env.JWT_SECRET || 'your-secret-key-change-in-production';

// Pricing per second for different plans
const PRICING = {
    basic: 0.15,
    professional: 0.25,
    enterprise: 0.35
};

// Middleware to verify JWT token
function authenticateToken(req, res, next) {
    const authHeader = req.headers['authorization'];
    const token = authHeader && authHeader.split(' ')[1];

    if (!token) {
        return res.status(401).json({ message: 'Access token required' });
    }

    jwt.verify(token, JWT_SECRET, (err, user) => {
        if (err) {
            return res.status(403).json({ message: 'Invalid or expired token' });
        }
        req.user = user;
        next();
    });
}

// Azure credentials
const credential = new DefaultAzureCredential();
const subscriptionId = process.env.AZURE_SUBSCRIPTION_ID;
const computeClient = new ComputeManagementClient(credential, subscriptionId);
const monitorClient = new MonitorClient(credential, subscriptionId);
const consumptionClient = new ConsumptionManagementClient(credential, subscriptionId);

// Helper function to get VM metrics
async function getVMMetrics(resourceId, metricNames) {
    try {
        const endTime = new Date();
        const startTime = new Date(endTime.getTime() - 60 * 60 * 1000); // Last hour

        const metrics = await monitorClient.metrics.list(resourceId, {
            timespan: `${startTime.toISOString()}/${endTime.toISOString()}`,
            interval: 'PT5M',
            metricnames: metricNames.join(','),
            aggregation: 'Average'
        });

        return metrics;
    } catch (error) {
        console.error('Error fetching metrics:', error);
        return null;
    }
}

// Helper function to get actual Azure costs
async function getAzureCosts() {
    try {
        const endDate = new Date();
        const startDate = new Date(endDate.getFullYear(), endDate.getMonth(), 1); // First day of current month
        
        // Get usage details for the current month
        const usageDetails = consumptionClient.usageDetails.list({
            scope: `/subscriptions/${subscriptionId}`,
            filter: `usageStart ge '${startDate.toISOString().split('T')[0]}' and usageEnd le '${endDate.toISOString().split('T')[0]}'`,
            top: 1000
        });

        let totalCost = 0;
        let vmCosts = {};

        for await (const usage of usageDetails) {
            if (usage.instanceName && usage.cost) {
                const vmName = usage.instanceName;
                const cost = parseFloat(usage.cost) || 0;
                
                if (!vmCosts[vmName]) {
                    vmCosts[vmName] = 0;
                }
                vmCosts[vmName] += cost;
                totalCost += cost;
            }
        }

        return {
            totalCost: Math.round(totalCost * 100) / 100, // Round to 2 decimal places
            vmCosts: vmCosts
        };
    } catch (error) {
        console.error('Error fetching Azure costs:', error);
        
        // If we can't get detailed costs, return estimated cost based on VM sizes
        return {
            totalCost: 0,
            vmCosts: {},
            estimated: true
        };
    }
}

// Helper function to estimate VM costs based on size and region
function estimateVMCost(vmSize, region, status) {
    // This is a simplified cost estimation - in production, you'd want to use Azure Pricing API
    const baseCosts = {
        'Standard_D2s_v3': { 'East US': 0.096, 'West US 2': 0.096, 'Central US': 0.096 },
        'Standard_D4s_v3': { 'East US': 0.192, 'West US 2': 0.192, 'Central US': 0.192 },
        'Standard_D8s_v3': { 'East US': 0.384, 'West US 2': 0.384, 'Central US': 0.384 },
        'Standard_D1s_v3': { 'East US': 0.048, 'West US 2': 0.048, 'Central US': 0.048 },
        'Standard_B1s': { 'East US': 0.0125, 'West US 2': 0.0125, 'Central US': 0.0125 },
        'Standard_B2s': { 'East US': 0.05, 'West US 2': 0.05, 'Central US': 0.05 }
    };

    const hourlyRate = baseCosts[vmSize]?.[region] || 0.1; // Default rate
    const hoursInMonth = 730; // Average hours in a month
    
    if (status === 'running') {
        return hourlyRate * hoursInMonth;
    } else {
        // Stopped VMs still incur storage costs (roughly 10% of compute cost)
        return hourlyRate * hoursInMonth * 0.1;
    }
}

// API Routes

// User Authentication Routes

// Signup
app.post('/api/auth/signup', async (req, res) => {
    try {
        const { fullName, email, password, plan } = req.body;

        // Validate input
        if (!fullName || !email || !password || !plan) {
            return res.status(400).json({ message: 'All fields are required' });
        }

        if (!PRICING[plan]) {
            return res.status(400).json({ message: 'Invalid plan selected' });
        }

        // Check if user already exists
        db.get('SELECT id FROM users WHERE email = ?', [email], async (err, row) => {
            if (err) {
                console.error('Database error:', err);
                return res.status(500).json({ message: 'Database error' });
            }

            if (row) {
                return res.status(400).json({ message: 'User with this email already exists' });
            }

            // Hash password
            const hashedPassword = await bcrypt.hash(password, 10);
            const userId = uuidv4();

            // Create user
            db.run(
                'INSERT INTO users (id, fullName, email, password, plan) VALUES (?, ?, ?, ?, ?)',
                [userId, fullName, email, hashedPassword, plan],
                function(err) {
                    if (err) {
                        console.error('Error creating user:', err);
                        return res.status(500).json({ message: 'Error creating user' });
                    }

                    // Generate JWT token
                    const token = jwt.sign(
                        { userId, email, fullName, plan },
                        JWT_SECRET,
                        { expiresIn: '7d' }
                    );

                    res.status(201).json({
                        message: 'User created successfully',
                        token,
                        user: {
                            id: userId,
                            fullName,
                            email,
                            plan
                        }
                    });
                }
            );
        });
    } catch (error) {
        console.error('Signup error:', error);
        res.status(500).json({ message: 'Internal server error' });
    }
});

// Login
app.post('/api/auth/login', async (req, res) => {
    try {
        const { email, password } = req.body;

        if (!email || !password) {
            return res.status(400).json({ message: 'Email and password are required' });
        }

        db.get('SELECT * FROM users WHERE email = ?', [email], async (err, user) => {
            if (err) {
                console.error('Database error:', err);
                return res.status(500).json({ message: 'Database error' });
            }

            if (!user) {
                return res.status(401).json({ message: 'Invalid credentials' });
            }

            // Check password
            const validPassword = await bcrypt.compare(password, user.password);
            if (!validPassword) {
                return res.status(401).json({ message: 'Invalid credentials' });
            }

            // Generate JWT token
            const token = jwt.sign(
                { userId: user.id, email: user.email, fullName: user.fullName, plan: user.plan },
                JWT_SECRET,
                { expiresIn: '7d' }
            );

            res.json({
                message: 'Login successful',
                token,
                user: {
                    id: user.id,
                    fullName: user.fullName,
                    email: user.email,
                    plan: user.plan
                }
            });
        });
    } catch (error) {
        console.error('Login error:', error);
        res.status(500).json({ message: 'Internal server error' });
    }
});

// Get user data
app.get('/api/user/data', authenticateToken, (req, res) => {
    const { userId } = req.user;
    const currentDate = new Date();
    const currentYear = currentDate.getFullYear();
    const currentMonth = currentDate.getMonth() + 1;

    db.get(
        `SELECT u.*, COALESCE(mu.totalSeconds, 0) as monthlyUsage, COALESCE(mu.totalCost, 0) as monthlyCost
         FROM users u
         LEFT JOIN monthly_usage mu ON u.id = mu.userId AND mu.year = ? AND mu.month = ?
         WHERE u.id = ?`,
        [currentYear, currentMonth, userId],
        (err, user) => {
            if (err) {
                console.error('Error fetching user data:', err);
                return res.status(500).json({ message: 'Database error' });
            }

            if (!user) {
                return res.status(404).json({ message: 'User not found' });
            }

            res.json({
                fullName: user.fullName,
                email: user.email,
                plan: user.plan,
                balance: user.balance,
                monthlyUsage: user.monthlyUsage,
                monthlyCost: user.monthlyCost
            });
        }
    );
});

// Usage tracking routes

// Start usage session
app.post('/api/usage/start', authenticateToken, (req, res) => {
    const { userId } = req.user;
    const sessionId = uuidv4();

    db.run(
        'INSERT INTO usage_sessions (id, userId, startTime) VALUES (?, ?, CURRENT_TIMESTAMP)',
        [sessionId, userId],
        function(err) {
            if (err) {
                console.error('Error starting usage session:', err);
                return res.status(500).json({ message: 'Error starting session' });
            }

            res.json({ 
                message: 'Usage session started',
                sessionId 
            });
        }
    );
});

// Stop usage session
app.post('/api/usage/stop', authenticateToken, (req, res) => {
    const { userId } = req.user;
    const { duration } = req.body;

    if (!duration || duration <= 0) {
        return res.status(400).json({ message: 'Valid duration required' });
    }

    // Get user's plan to calculate cost
    db.get('SELECT plan FROM users WHERE id = ?', [userId], (err, user) => {
        if (err) {
            console.error('Error fetching user plan:', err);
            return res.status(500).json({ message: 'Database error' });
        }

        if (!user) {
            return res.status(404).json({ message: 'User not found' });
        }

        const cost = duration * PRICING[user.plan];
        const sessionId = uuidv4();

        // Record the session
        db.run(
            `INSERT INTO usage_sessions (id, userId, endTime, duration, cost) 
             VALUES (?, ?, CURRENT_TIMESTAMP, ?, ?)`,
            [sessionId, userId, duration, cost],
            function(err) {
                if (err) {
                    console.error('Error recording usage session:', err);
                    return res.status(500).json({ message: 'Error recording session' });
                }

                // Update monthly usage
                updateMonthlyUsage(userId, duration, cost);

                res.json({ 
                    message: 'Usage session recorded',
                    duration,
                    cost: cost.toFixed(2)
                });
            }
        );
    });
});

// Get usage history
app.get('/api/user/usage', authenticateToken, (req, res) => {
    const { userId } = req.user;
    const limit = parseInt(req.query.limit) || 50;

    db.all(
        `SELECT startTime, endTime, duration, cost 
         FROM usage_sessions 
         WHERE userId = ? 
         ORDER BY startTime DESC 
         LIMIT ?`,
        [userId, limit],
        (err, sessions) => {
            if (err) {
                console.error('Error fetching usage history:', err);
                return res.status(500).json({ message: 'Database error' });
            }

            res.json(sessions);
        }
    );
});

// Helper function to update monthly usage
function updateMonthlyUsage(userId, duration, cost) {
    const currentDate = new Date();
    const currentYear = currentDate.getFullYear();
    const currentMonth = currentDate.getMonth() + 1;

    db.run(
        `INSERT OR REPLACE INTO monthly_usage (id, userId, year, month, totalSeconds, totalCost)
         SELECT 
             COALESCE(existing.id, ?) as id,
             ? as userId,
             ? as year,
             ? as month,
             COALESCE(existing.totalSeconds, 0) + ? as totalSeconds,
             COALESCE(existing.totalCost, 0) + ? as totalCost
         FROM monthly_usage existing
         WHERE existing.userId = ? AND existing.year = ? AND existing.month = ?
         UNION ALL
         SELECT ?, ?, ?, ?, ?, ?, ?, ?, ?
         WHERE NOT EXISTS (
             SELECT 1 FROM monthly_usage 
             WHERE userId = ? AND year = ? AND month = ?
         )`,
        [
            uuidv4(), userId, currentYear, currentMonth, duration, cost, userId, currentYear, currentMonth,
            uuidv4(), userId, currentYear, currentMonth, duration, cost, userId, currentYear, currentMonth,
            userId, currentYear, currentMonth
        ],
        function(err) {
            if (err) {
                console.error('Error updating monthly usage:', err);
            }
        }
    );
}

// Get all VMs
app.get('/api/vms', async (req, res) => {
    try {
        const vms = [];
        
        // Get all resource groups
        const resourceGroups = computeClient.resourceGroups.list();
        
        for await (const rg of resourceGroups) {
            // Get VMs in each resource group
            const vmList = computeClient.virtualMachines.list(rg.name);
            
            for await (const vm of vmList) {
                try {
                    // Get VM instance view for status
                    const instanceView = await computeClient.virtualMachines.instanceView(rg.name, vm.name);
                    
                    // Get metrics
                    const resourceId = `/subscriptions/${subscriptionId}/resourceGroups/${rg.name}/providers/Microsoft.Compute/virtualMachines/${vm.name}`;
                    const metrics = await getVMMetrics(resourceId, ['Percentage CPU', 'Available Memory Bytes', 'Disk Read Bytes', 'Disk Write Bytes', 'Network In Total', 'Network Out Total']);
                    
                    // Calculate uptime
                    const powerState = instanceView.statuses?.find(s => s.code?.startsWith('PowerState/'));
                    const uptime = powerState?.code === 'PowerState/running' ? 
                        new Date() - new Date(instanceView.statuses?.find(s => s.code === 'OSState/generalized')?.time || Date.now()) : 
                        '0d 0h 0m';
                    
                    // Parse metrics
                    let cpu = 0, memory = 0, disk = 0, network = 0;
                    
                    if (metrics && metrics.value) {
                        const cpuMetric = metrics.value.find(m => m.name.value === 'Percentage CPU');
                        const memoryMetric = metrics.value.find(m => m.name.value === 'Available Memory Bytes');
                        const diskMetric = metrics.value.find(m => m.name.value === 'Disk Read Bytes' || m.name.value === 'Disk Write Bytes');
                        const networkMetric = metrics.value.find(m => m.name.value === 'Network In Total' || m.name.value === 'Network Out Total');
                        
                        if (cpuMetric && cpuMetric.timeseries && cpuMetric.timeseries[0]?.data) {
                            const recentData = cpuMetric.timeseries[0].data.filter(d => d.average !== null).slice(-5);
                            cpu = recentData.length > 0 ? Math.round(recentData.reduce((sum, d) => sum + d.average, 0) / recentData.length) : 0;
                        }
                        
                        if (memoryMetric && memoryMetric.timeseries && memoryMetric.timeseries[0]?.data) {
                            const recentData = memoryMetric.timeseries[0].data.filter(d => d.average !== null).slice(-5);
                            const avgMemory = recentData.length > 0 ? recentData.reduce((sum, d) => sum + d.average, 0) / recentData.length : 0;
                            // Convert to percentage (assuming 8GB default, adjust as needed)
                            memory = Math.round((1 - avgMemory / (8 * 1024 * 1024 * 1024)) * 100);
                        }
                        
                        if (diskMetric && diskMetric.timeseries && diskMetric.timeseries[0]?.data) {
                            const recentData = diskMetric.timeseries[0].data.filter(d => d.average !== null).slice(-5);
                            const avgDisk = recentData.length > 0 ? recentData.reduce((sum, d) => sum + d.average, 0) / recentData.length : 0;
                            disk = Math.round(avgDisk / (1024 * 1024)); // Convert to MB/s
                        }
                        
                        if (networkMetric && networkMetric.timeseries && networkMetric.timeseries[0]?.data) {
                            const recentData = networkMetric.timeseries[0].data.filter(d => d.average !== null).slice(-5);
                            const avgNetwork = recentData.length > 0 ? recentData.reduce((sum, d) => sum + d.average, 0) / recentData.length : 0;
                            network = Math.round(avgNetwork / (1024 * 1024)); // Convert to MB/s
                        }
                    }
                    
                    vms.push({
                        id: vm.id,
                        name: vm.name,
                        status: powerState?.code?.replace('PowerState/', '') || 'unknown',
                        cpu: Math.max(0, Math.min(100, cpu)),
                        memory: Math.max(0, Math.min(100, memory)),
                        disk: disk,
                        network: network,
                        region: vm.location,
                        size: vm.hardwareProfile?.vmSize || 'Unknown',
                        resourceGroup: rg.name,
                        uptime: typeof uptime === 'number' ? 
                            `${Math.floor(uptime / (1000 * 60 * 60 * 24))}d ${Math.floor((uptime % (1000 * 60 * 60 * 24)) / (1000 * 60 * 60))}h ${Math.floor((uptime % (1000 * 60 * 60)) / (1000 * 60))}m` : 
                            uptime
                    });
                } catch (error) {
                    console.error(`Error processing VM ${vm.name}:`, error);
                }
            }
        }
        
        res.json(vms);
    } catch (error) {
        console.error('Error fetching VMs:', error);
        res.status(500).json({ error: 'Failed to fetch VMs' });
    }
});

// VM Control Routes
app.post('/api/vms/:resourceGroup/:vmName/start', async (req, res) => {
    try {
        const { resourceGroup, vmName } = req.params;
        await computeClient.virtualMachines.beginStart(resourceGroup, vmName);
        res.json({ message: 'VM start initiated' });
    } catch (error) {
        console.error('Error starting VM:', error);
        res.status(500).json({ error: 'Failed to start VM' });
    }
});

app.post('/api/vms/:resourceGroup/:vmName/stop', async (req, res) => {
    try {
        const { resourceGroup, vmName } = req.params;
        await computeClient.virtualMachines.beginDeallocate(resourceGroup, vmName);
        res.json({ message: 'VM stop initiated' });
    } catch (error) {
        console.error('Error stopping VM:', error);
        res.status(500).json({ error: 'Failed to stop VM' });
    }
});

app.post('/api/vms/:resourceGroup/:vmName/restart', async (req, res) => {
    try {
        const { resourceGroup, vmName } = req.params;
        await computeClient.virtualMachines.beginRestart(resourceGroup, vmName);
        res.json({ message: 'VM restart initiated' });
    } catch (error) {
        console.error('Error restarting VM:', error);
        res.status(500).json({ error: 'Failed to restart VM' });
    }
});

// Get Azure costs
app.get('/api/costs', async (req, res) => {
    try {
        const costs = await getAzureCosts();
        res.json(costs);
    } catch (error) {
        console.error('Error fetching costs:', error);
        res.status(500).json({ error: 'Failed to fetch costs' });
    }
});

// Serve static files
app.use(express.static('.'));

// Health check
app.get('/api/health', (req, res) => {
    res.json({ status: 'OK', timestamp: new Date().toISOString() });
});

// Start server
app.listen(PORT, () => {
    console.log(`Server running on port ${PORT}`);
    console.log(`Health check: http://localhost:${PORT}/api/health`);
});

// Graceful shutdown
process.on('SIGINT', () => {
    console.log('Shutting down server...');
    db.close((err) => {
        if (err) {
            console.error('Error closing database:', err);
        } else {
            console.log('Database connection closed.');
        }
        process.exit(0);
    });
}); 