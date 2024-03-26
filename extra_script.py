Import("env")

def before_upload(source, target, env):
    env.Execute("killall com")

env.AddPreAction("upload", before_upload)
