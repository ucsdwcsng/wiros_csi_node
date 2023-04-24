ssh $1@$2 rm -rf /jffs/csi
scp -r csi $1@$2:/jffs/
