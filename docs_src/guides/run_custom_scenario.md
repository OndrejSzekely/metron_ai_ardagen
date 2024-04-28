# Run Custom Scenario

1. In `conf/writer.yaml` config set `fs_store_path` to a folder (absolute path) where generated images will be stored.

2. *Dummy Scenario* is a default *Scenario* set for *ArDaGen* configuration. Go into `conf/metron_ai_ardagen_config.yaml`
   file and replace `dummy_scenario` with the new *Scenario*.

3. Go into *Isaac Sim* root folder in terminal and run following command:

```shell
./python.sh  metron_ai_ardagen/metron_ai_ardagen_run.py
```

3. Find generated images in the `fs_store_path` folder.
