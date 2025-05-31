#!/bin/bash
# auditoria_basica.sh - Auditoría básica de seguridad en Linux

echo "==== Auditoría Básica de Seguridad en Linux ===="

# 1. Información del sistema
echo -e "\n[+] Información del sistema:"
uname -a
uptime
hostnamectl

# 2. Usuarios conectados
echo -e "\n[+] Usuarios conectados actualmente:"
who

# 3. Últimos logins
echo -e "\n[+] Últimos accesos:"
last -n 5

# 4. Intentos de acceso fallidos"
echo -e "\n[+] Intentos fallidos de login:"
lastb -n 5 2>/dev/null || echo "No hay registros o falta el archivo /var/log/btmp"

# 5. Revisión de usuarios con UID 0"
echo -e "\n[+] Usuarios con privilegios de root (UID 0):"
awk -F: '$3 == 0 {print $1}' /etc/passwd

# 6. Archivos con permisos SUID y SGID
echo -e "\n[+] Archivos con SUID:"
find / -perm -4000 -type f 2>/dev/null

echo -e "\n[+] Archivos con SGID:"
find / -perm -2000 -type f 2>/dev/null

# 7. Estado del firewall
echo -e "\n[+] Estado del firewall UFW:"
ufw status verbose

# 8. Puertos abiertos
echo -e "\n[+] Puertos abiertos (ss):"
ss -tuln

# 9. Servicios activos
echo -e "\n[+] Servicios activos:"
systemctl list-units --type=service --state=running

# 10. Verifica si se han instalado herramientas básicas de seguridad
echo -e "\n[+] Verificación de herramientas de seguridad:"
for tool in lynis auditd chkrootkit rkhunter; do
  command -v $tool >/dev/null && echo "$tool: ✅ instalado" || echo "$tool: ❌ no instalado"
done

echo -e "\nAuditoría básica completada."
