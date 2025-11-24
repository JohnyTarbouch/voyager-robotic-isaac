#!/bin/bash
# Initialize Git repository and push to GitHub

echo "🚀 Initializing Git repository..."

# Initialize git if not already done
if [ ! -d .git ]; then
    git init
    echo "✓ Git repository initialized"
else
    echo "✓ Git repository already exists"
fi

# Create necessary directories
mkdir -p logs data

# Create empty __init__.py files
touch src/__init__.py
touch tests/__init__.py

# Add all files
git add .

# Create initial commit
git commit -m "Initial commit: Voyager-style Jetbot agent for Isaac Sim

Features:
- Direct robot control via socket API
- Manual control CLI
- Skill library with SQLite
- Comprehensive logging
- Modular architecture
- Ready for LLM integration
"

echo ""
echo "✓ Initial commit created"
echo ""
echo "📝 Next steps:"
echo ""
echo "1. Create a new repository on GitHub:"
echo "   https://github.com/new"
echo ""
echo "2. Name it: voyager-jetbot-isaac"
echo ""
echo "3. Don't initialize with README (we already have one)"
echo ""
echo "4. Run these commands to push:"
echo ""
echo "   git branch -M main"
echo "   git remote add origin https://github.com/YOUR_USERNAME/voyager-jetbot-isaac.git"
echo "   git push -u origin main"
echo ""
echo "Replace YOUR_USERNAME with your GitHub username"
echo ""
echo "🎉 Done! Your project is ready for GitHub!"
