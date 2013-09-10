'''

    This file is part of GEAR.

    GEAR is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/lgpl.html>.

    Author:     Jeremie Passerin      geerem@hotmail.com
    Company:    Studio Nest (TM)
    Date:       2010 / 11 / 15

'''

## @package gear.xsi.plugin
# @author Jeremie Passerin
#

##########################################################
# GLOBAL
##########################################################
from gear.xsi import xsi

##########################################################
# PLUGINS
##########################################################
# PluginExists ===========================================
## Check if given plugin name exists
# @param name String - Plugin Name.
# @return Boolean - True if plugin exists
def pluginExists(names, all=True):

    if isinstance(names, str):
        names = [names]

    exists = True
    for name in names:

        plugin = xsi.Plugins(name)

        if plugin:
            if not all:
                return True
        else:
            if all:
                return False
            else:
                exists = False

    return exists

# GetPluginPath ==========================================
## Return the path of given plugin name
# @param name String - Plugin Name.
# @return String - plugin path. False if plugin wasn't found.
def getPluginPath(name):

    plugin = xsi.Plugins(name)
    if plugin:
        return plugin.OriginPath
    else:
        return False

# GetPluginFilePath ======================================
## Return the full path (including filename) of given plugin name
# @param name String - Plugin Name.
# @return String - plugin full path. False if plugin wasn't found.
def getPluginFullPath(name):

    plugin = xsi.Plugins(name)
    if plugin:
        return plugin.FileName
    else:
        return False
