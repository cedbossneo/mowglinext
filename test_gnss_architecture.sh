#!/bin/bash
# Test script to demonstrate the modular GNSS backend architecture

echo "=== Test de l'architecture modulaire GNSS ==="
echo

echo "1. Test avec gnss_backend=none (défaut):"
echo "   ros2 launch mowgli_bringup full_system.launch.py gnss_backend:=none"
echo "   → Aucun backend GNSS chargé"
echo

echo "2. Test avec gnss_backend=ublox:"
echo "   ros2 launch mowgli_bringup full_system.launch.py gnss_backend:=ublox"
echo "   → Charge uniquement le backend u-blox"
echo

echo "3. Test avec gnss_backend=unicore:"
echo "   ros2 launch mowgli_bringup full_system.launch.py gnss_backend:=unicore"
echo "   → Charge uniquement le backend Unicore"
echo

echo "4. Arguments individuels toujours disponibles (legacy):"
echo "   ros2 launch mowgli_bringup full_system.launch.py enable_ublox_gnss:=true"
echo "   → Charge le backend u-blox (méthode legacy)"
echo

echo "=== Avantages de cette architecture ==="
echo "✅ Séparation claire des backends"
echo "✅ Un seul backend actif à la fois"
echo "✅ Pas de conflit entre backends"
echo "✅ Interface commune /gnss/* respectée"
echo "✅ Configuration centralisée via gnss_backend"
echo "✅ Rétrocompatibilité avec les anciens arguments"