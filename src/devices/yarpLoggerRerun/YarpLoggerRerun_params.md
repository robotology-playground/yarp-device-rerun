| Group name      | Parameter name        | Type            | Units   | Default Value       | Required | Description                                    | Notes                            |
|:---------------:|:---------------------:|:---------------:|:-------:|:----------------:   |:--------:|:----------------------------------------------:|:--------------------------------:|
|                 | axesNames             | vector<string>  | -       |                     |  Yes     |                                                |                                  |
|                 | logIEncoders          | bool            | -       |     true            |  No      |                                                |                                  |
|                 | logIMotorEncoders     | bool            | -       |     false           |  No      |                                                |                                  |
|                 | logIPidControl        | bool            | -       |     false           |  No      |                                                |                                  |
|                 | logITorqueControl     | bool            | -       |     false           |  No      |                                                |                                  |
|                 | logIAmplifierControl  | bool            | -       |     false           |  No      |                                                |                                  |
|                 | logIControlMode       | bool            | -       |     false           |  No      |                                                |                                  |
|                 | logIInteractionMode   | bool            | -       |     false           |  No      |                                                |                                  |
|                 | logIMotorTemperatures | bool            | -       |     false           |  No      |                                                |                                  |
|                 | logILocalization2D    | bool            | -       |     false           |  No      |                                                |                                  |
|                 | localizationRemoteName| string          | -       | /localizationRemote |  No      | Must have the name of the remote port defined in the related nws |                |
|                 | logIRawValuesPublisher| bool            | -       |     false           |  No      |                                                |                                  |
|                 | logURDF               | bool            | -       |     false           |  No      |                                                |                                  |
|                 | fileName              | string          | -       |     log_test        |  No      |                                                |                                  |
|                 | filePath              | string          | -       |/home/ergocub/test   |  No      |                                                |                                  |
|                 | saveToFile            | bool            | -       |     false           |  No      |                                                |                                  |
|                 | viewerIp              | string          | -       |    localhost        |  No      |                                                |                                  |