// The Lotse dashboard logic. It loads the map once, draws it and the robots on a
// canvas, keeps the order list and throughput current over HTTP and a WebSocket,
// and posts new orders. Plain browser JavaScript, no build step, no imports.

const canvas = document.getElementById('map');
const context = canvas.getContext('2d');
const pickupSelect = document.getElementById('pickup');
const dropoffSelect = document.getElementById('dropoff');
const orderForm = document.getElementById('order-form');
const formMessage = document.getElementById('form-message');
const orderRows = document.querySelector('#orders tbody');
const throughputList = document.getElementById('throughput');
const statusLabel = document.getElementById('status');

let map = null;
let robots = [];
let orders = [];

// Scales the fixed canvas so one grid cell is an equal number of pixels.
// returns {size, offsetX, offsetY} for the current map
function cellGeometry() {
    const size = Math.min(canvas.width / map.width, canvas.height / map.height);
    return {
        size: size,
        offsetX: (canvas.width - size * map.width) / 2,
        offsetY: (canvas.height - size * map.height) / 2
    };
}

// Draws the static warehouse once: free cells, shelves, stations and parking.
function drawMap() {
    const geometry = cellGeometry();
    context.fillStyle = '#12161c';
    context.fillRect(0, 0, canvas.width, canvas.height);
    for (let y = 0; y < map.height; y += 1) {
        for (let x = 0; x < map.width; x += 1) {
            const wall = map.rows[y][x] === '#';
            context.fillStyle = wall ? '#55606f' : '#2a3442';
            context.fillRect(geometry.offsetX + x * geometry.size,
                geometry.offsetY + y * geometry.size,
                geometry.size - 1, geometry.size - 1);
        }
    }
    for (const station of map.stations) {
        context.fillStyle = station.name.startsWith('pickup') ? '#52c98a' : '#f0a34a';
        context.fillRect(geometry.offsetX + station.x * geometry.size + 2,
            geometry.offsetY + station.y * geometry.size + 2,
            geometry.size - 5, geometry.size - 5);
    }
    for (const cell of map.parking) {
        context.strokeStyle = '#6f7f92';
        context.strokeRect(geometry.offsetX + cell.x * geometry.size + 2,
            geometry.offsetY + cell.y * geometry.size + 2,
            geometry.size - 5, geometry.size - 5);
    }
}

// Draws every robot at its interpolated position, plus its target.
function drawRobots() {
    const geometry = cellGeometry();
    context.font = Math.max(9, geometry.size * 0.5) + 'px monospace';
    context.textAlign = 'center';
    context.textBaseline = 'middle';
    for (const robot of robots) {
        const progress = robot.target_x === robot.x && robot.target_y === robot.y
            ? 0 : robot.progress;
        const cellX = robot.x + (robot.target_x - robot.x) * progress;
        const cellY = robot.y + (robot.target_y - robot.y) * progress;
        const centreX = geometry.offsetX + (cellX + 0.5) * geometry.size;
        const centreY = geometry.offsetY + (cellY + 0.5) * geometry.size;
        context.beginPath();
        context.arc(centreX, centreY, geometry.size * 0.35, 0, Math.PI * 2);
        context.fillStyle = robot.busy ? '#56c1ff' : '#3a4a5c';
        context.fill();
        if (robot.stopped) {
            context.strokeStyle = '#f0a34a';
            context.lineWidth = 2;
            context.stroke();
        }
        context.fillStyle = '#12161c';
        context.fillText(String(robot.id), centreX, centreY);
    }
}

// Redraws the map and the robots together.
function render() {
    if (!map) {
        return;
    }
    drawMap();
    drawRobots();
}

// Fetches the map, draws it and fills the station selects.
async function loadMap() {
    const response = await fetch('/map');
    map = await response.json();
    pickupSelect.innerHTML = '';
    dropoffSelect.innerHTML = '';
    for (const station of map.stations) {
        const option = document.createElement('option');
        option.value = station.name;
        option.textContent = station.name;
        if (station.name.startsWith('pickup')) {
            pickupSelect.appendChild(option);
        } else if (station.name.startsWith('dropoff')) {
            dropoffSelect.appendChild(option.cloneNode(true));
        }
    }
    render();
}

// Renders the order table.
function renderOrders() {
    orderRows.innerHTML = '';
    for (const order of orders) {
        const row = document.createElement('tr');
        const fields = [order.id, order.state, order.robot < 0 ? '-' : order.robot,
            '(' + order.pickup.x + ',' + order.pickup.y + ')',
            '(' + order.dropoff.x + ',' + order.dropoff.y + ')'];
        for (const value of fields) {
            const cell = document.createElement('td');
            cell.textContent = value;
            row.appendChild(cell);
        }
        const action = document.createElement('td');
        if (['pending', 'assigned'].includes(order.state)) {
            const button = document.createElement('button');
            button.className = 'cancel';
            button.textContent = 'cancel';
            button.onclick = () => cancelOrder(order.id);
            action.appendChild(button);
        }
        row.appendChild(action);
        orderRows.appendChild(row);
    }
}

// Fetches and renders the order list.
async function loadOrders() {
    const response = await fetch('/orders');
    orders = await response.json();
    renderOrders();
}

// Fetches and renders the throughput counters.
async function loadThroughput() {
    const response = await fetch('/throughput');
    const data = await response.json();
    throughputList.innerHTML = '';
    const rows = [['completed', data.orders_completed], ['open', data.orders_open],
        ['cancelled', data.orders_cancelled], ['failed', data.orders_failed],
        ['orders / hour', Number(data.orders_per_hour).toFixed(1)],
        ['planning calls', data.planning_calls],
        ['plan failures', data.planning_failures]];
    for (const [label, value] of rows) {
        const term = document.createElement('dt');
        term.textContent = label;
        const detail = document.createElement('dd');
        detail.textContent = value;
        throughputList.appendChild(term);
        throughputList.appendChild(detail);
    }
}

// Submits the form as a JSON order.
async function submitOrder(event) {
    event.preventDefault();
    formMessage.textContent = '';
    const body = JSON.stringify({
        pickup: pickupSelect.value,
        dropoff: dropoffSelect.value,
        priority: Number(document.getElementById('priority').value)
    });
    const response = await fetch('/orders', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: body
    });
    const data = await response.json();
    formMessage.textContent = response.ok
        ? 'order ' + data.id + ' accepted'
        : data.error;
    if (response.ok) {
        loadOrders();
        loadThroughput();
    }
}

// Cancels one order and shows the reason when it is refused.
async function cancelOrder(id) {
    const response = await fetch('/orders/' + id, {method: 'DELETE'});
    if (!response.ok) {
        const data = await response.json();
        formMessage.textContent = data.error;
    }
    loadOrders();
    loadThroughput();
}

// Opens the live feed and applies fleet and order updates.
function connect() {
    const scheme = location.protocol === 'https:' ? 'wss://' : 'ws://';
    const socket = new WebSocket(scheme + location.host + '/ws');
    socket.onopen = () => {
        statusLabel.textContent = 'live';
    };
    socket.onclose = () => {
        statusLabel.textContent = 'disconnected, retrying';
        setTimeout(connect, 1000);
    };
    socket.onmessage = (event) => {
        const message = JSON.parse(event.data);
        if (message.type === 'fleet') {
            robots = message.robots;
            statusLabel.textContent = message.deadlocked
                ? 'deadlock detected'
                : message.robots.length + ' robots';
            render();
        } else if (message.type === 'order') {
            loadOrders();
            loadThroughput();
        }
    };
}

// Wires the form and starts the dashboard.
async function start() {
    orderForm.addEventListener('submit', submitOrder);
    await loadMap();
    await loadOrders();
    await loadThroughput();
    connect();
    setInterval(loadThroughput, 2000);
}

start();
