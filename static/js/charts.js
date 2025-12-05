const ctxAlt = document.getElementById('altitudeChart').getContext('2d');
const ctxVel = document.getElementById('velocityChart').getContext('2d');

const commonOptions = {
    responsive: true,
    maintainAspectRatio: false,
    animation: false, // Disable animation for performance
    interaction: {
        mode: 'index',
        intersect: false,
    },
    plugins: {
        legend: {
            display: true,
            labels: { color: '#8b949e' }
        }
    },
    scales: {
        x: {
            grid: { color: '#30363d' },
            ticks: { color: '#8b949e' }
        },
        y: {
            grid: { color: '#30363d' },
            ticks: { color: '#8b949e' }
        }
    }
};

const altitudeChart = new Chart(ctxAlt, {
    type: 'line',
    data: {
        labels: [],
        datasets: [{
            label: 'Altitude (m)',
            data: [],
            borderColor: '#58a6ff',
            backgroundColor: 'rgba(88, 166, 255, 0.1)',
            borderWidth: 2,
            fill: true,
            tension: 0.4,
            pointRadius: 0
        }]
    },
    options: commonOptions
});

const velocityChart = new Chart(ctxVel, {
    type: 'line',
    data: {
        labels: [],
        datasets: [{
            label: 'Velocity (m/s)',
            data: [],
            borderColor: '#238636',
            backgroundColor: 'rgba(35, 134, 54, 0.1)',
            borderWidth: 2,
            fill: true,
            tension: 0.4,
            pointRadius: 0
        }]
    },
    options: commonOptions
});

// Update function called from main.js
window.updateCharts = function (data) {
    const timeLabel = data.time.toFixed(1);

    // Add new data
    altitudeChart.data.labels.push(timeLabel);
    altitudeChart.data.datasets[0].data.push(data.altitude);

    velocityChart.data.labels.push(timeLabel);
    velocityChart.data.datasets[0].data.push(data.velocity);

    // Keep only last 50 points to prevent memory leak / lag
    if (altitudeChart.data.labels.length > 50) {
        altitudeChart.data.labels.shift();
        altitudeChart.data.datasets[0].data.shift();

        velocityChart.data.labels.shift();
        velocityChart.data.datasets[0].data.shift();
    }

    altitudeChart.update('none'); // 'none' mode for performance
    velocityChart.update('none');
};

// Theme Update
window.updateChartTheme = function (theme) {
    const textColor = theme === 'dark' ? '#8b949e' : '#57606a';
    const gridColor = theme === 'dark' ? '#30363d' : '#d0d7de';

    [altitudeChart, velocityChart].forEach(chart => {
        chart.options.plugins.legend.labels.color = textColor;
        chart.options.scales.x.grid.color = gridColor;
        chart.options.scales.x.ticks.color = textColor;
        chart.options.scales.y.grid.color = gridColor;
        chart.options.scales.y.ticks.color = textColor;
        chart.update();
    });
};
