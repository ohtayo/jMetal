# jMetalでパッケージ化したjarファイルを1つのフォルダにコピーするスクリプト

# import
import shutil

# フォルダとjar名の定義
folder = 'C:\workspace\jMetal'
files = ['problem', 'exec', 'core', 'algorithm']
lib_folder = 'C:\workspace\jMetal\jMetal-problem\lib'
lib_files = ['commons', 'building']

# jmetalのjarのコピー
for file in files:
    fromfile = folder+'\\jmetal-'+file+'\\target\jmetal-'+file+'-6.0-SNAPSHOT-jar-with-dependencies.jar'
    tofile = folder+'\package\jmetal-'+file+'.jar' # バージョン部分をリネーム
    shutil.copy2(fromfile, tofile)

# libのjarのコピー
for lib_file in lib_files:
    fromfile = lib_folder+'\\'+lib_file+'-java.jar'
    tofile = folder+'\package\\'+lib_file+'-java.jar'
    shutil.copy2(fromfile, tofile)

