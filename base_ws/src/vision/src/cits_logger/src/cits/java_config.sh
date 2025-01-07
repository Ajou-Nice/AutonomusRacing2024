sudo update-alternatives --install "/usr/bin/java" "java" "/usr/local/java/jdk1.8.0_202/bin/java" 1
sudo update-alternatives --install "/usr/bin/javac" "javac" "/usr/local/java/jdk1.8.0_202/bin/javac" 1
sudo update-alternatives --install "/usr/bin/javaws" "javaws" "/usr/local/java/jdk1.8.0_202/bin/javaws" 1
sudo update-alternatives --set java /usr/local/java/jdk1.8.0_202/bin/java
sudo update-alternatives --set javac /usr/local/java/jdk1.8.0_202/bin/javac
sudo update-alternatives --set javaws /usr/local/java/jdk1.8.0_202/bin/javaws