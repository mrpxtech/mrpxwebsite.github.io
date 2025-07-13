require('dotenv').config();
const express = require('express');
const cors = require('cors');
const helmet = require('helmet');
const rateLimit = require('express-rate-limit');
const { DefaultAzureCredential } = require('@azure/identity');
const { ComputeManagementClient } = require('@azure/arm-compute');
const { MonitorClient } = require('@azure/arm-monitor');

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

// Azure credentials
const credential = new DefaultAzureCredential();
const subscriptionId = process.env.AZURE_SUBSCRIPTION_ID;
const computeClient = new ComputeManagementClient(credential, subscriptionId);
const monitorClient = new MonitorClient(credential, subscriptionId);

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

// API Routes

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
                    // Add VM with basic info if detailed processing fails
                    vms.push({
                        id: vm.id,
                        name: vm.name,
                        status: 'unknown',
                        cpu: 0,
                        memory: 0,
                        disk: 0,
                        network: 0,
                        region: vm.location,
                        size: vm.hardwareProfile?.vmSize || 'Unknown',
                        resourceGroup: rg.name,
                        uptime: '0d 0h 0m'
                    });
                }
            }
        }
        
        res.json(vms);
    } catch (error) {
        console.error('Error fetching VMs:', error);
        res.status(500).json({ error: 'Failed to fetch VMs' });
    }
});

// Get VM summary statistics
app.get('/api/vms/summary', async (req, res) => {
    try {
        const vmsResponse = await fetch(`${req.protocol}://${req.get('host')}/api/vms`);
        const vms = await vmsResponse.json();
        
        const runningVMs = vms.filter(vm => vm.status === 'running');
        const stoppedVMs = vms.filter(vm => vm.status === 'stopped');
        
        const avgCPU = runningVMs.length > 0 ? 
            Math.round(runningVMs.reduce((sum, vm) => sum + vm.cpu, 0) / runningVMs.length) : 0;
        const avgMemory = runningVMs.length > 0 ? 
            Math.round(runningVMs.reduce((sum, vm) => sum + vm.memory, 0) / runningVMs.length) : 0;
        
        // Estimate monthly cost (rough calculation)
        const estimatedCost = vms.reduce((total, vm) => {
            // This is a simplified cost calculation - adjust based on your actual pricing
            const baseCost = vm.size.includes('D2') ? 50 : vm.size.includes('D4') ? 100 : vm.size.includes('D8') ? 200 : 75;
            return total + (vm.status === 'running' ? baseCost : baseCost * 0.1); // 10% for stopped VMs
        }, 0);
        
        res.json({
            totalVMs: vms.length,
            runningVMs: runningVMs.length,
            stoppedVMs: stoppedVMs.length,
            avgCPU: avgCPU,
            avgMemory: avgMemory,
            estimatedMonthlyCost: estimatedCost
        });
    } catch (error) {
        console.error('Error fetching summary:', error);
        res.status(500).json({ error: 'Failed to fetch summary' });
    }
});

// Start VM
app.post('/api/vms/:resourceGroup/:vmName/start', async (req, res) => {
    try {
        const { resourceGroup, vmName } = req.params;
        await computeClient.virtualMachines.beginStartAndWait(resourceGroup, vmName);
        res.json({ message: 'VM started successfully' });
    } catch (error) {
        console.error('Error starting VM:', error);
        res.status(500).json({ error: 'Failed to start VM' });
    }
});

// Stop VM
app.post('/api/vms/:resourceGroup/:vmName/stop', async (req, res) => {
    try {
        const { resourceGroup, vmName } = req.params;
        await computeClient.virtualMachines.beginDeallocateAndWait(resourceGroup, vmName);
        res.json({ message: 'VM stopped successfully' });
    } catch (error) {
        console.error('Error stopping VM:', error);
        res.status(500).json({ error: 'Failed to stop VM' });
    }
});

// Restart VM
app.post('/api/vms/:resourceGroup/:vmName/restart', async (req, res) => {
    try {
        const { resourceGroup, vmName } = req.params;
        await computeClient.virtualMachines.beginRestartAndWait(resourceGroup, vmName);
        res.json({ message: 'VM restarted successfully' });
    } catch (error) {
        console.error('Error restarting VM:', error);
        res.status(500).json({ error: 'Failed to restart VM' });
    }
});

// Health check
app.get('/api/health', (req, res) => {
    res.json({ status: 'healthy', timestamp: new Date().toISOString() });
});

app.listen(PORT, () => {
    console.log(`Server running on port ${PORT}`);
    console.log(`Health check: http://localhost:${PORT}/api/health`);
    console.log(`VMs API: http://localhost:${PORT}/api/vms`);
}); 