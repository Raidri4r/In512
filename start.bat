@echo off
REM Script pour lancer le serveur et les 2 agents automatiquement

echo Lancement du serveur...
start "Serveur" cmd /k "cd /d %~dp0 && python scripts/server.py -nb 4 -m 3"

REM Attendre un peu que le serveur demarre
timeout /t 2 /nobreak > nul

echo Lancement de l'agent 1...
start "Agent 0" cmd /k "cd /d %~dp0 && python scripts/agent.py"

timeout /t 1 /nobreak > nul

echo Lancement de l'agent 2...
start "Agent 1" cmd /k "cd /d %~dp0 && python scripts/agent.py"

echo Lancement de l'agent 2...
start "Agent 2" cmd /k "cd /d %~dp0 && python scripts/agent.py"

echo Lancement de l'agent 2...
start "Agent 3" cmd /k "cd /d %~dp0 && python scripts/agent.py"


echo.
echo Tous les processus sont lances !
echo Fermez cette fenetre quand vous avez fini.
