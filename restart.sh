#!/usr/bin/env bash
set -e

# ArduPilot SITL + Webots Simulation Restart Script

echo "🔄 Restarting ArduPilot SITL + Webots simulation..."

# 1. Verify scripts exist
if [ ! -f "./stop.sh" ] || [ ! -f "./start.sh" ]; then
    echo "❌ Error: 'stop.sh' or 'start.sh' not found."
    echo "   Please ensure you are running this from the directory containing these scripts."
    exit 1
fi

# 2. Make sure they are executable (just in case)
chmod +x ./stop.sh ./start.sh

# 3. Stop the containers
./stop.sh

# 4. Wait briefly to ensure ports (like 6080 and 14550) are fully released
echo "⏳ Waiting for ports to release..."
sleep 3

# 5. Start the containers
./start.sh
