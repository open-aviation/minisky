import json

import numpy as np
import pandas as pd

import minisky
from minisky.tools import geo
from minisky.tools.aero import nm


def findall(lst, x):
    """Find indices of multiple occurences of x in lst."""
    idx = []
    i = 0
    found = True
    while i < len(lst) and found:
        try:
            i = lst[i:].index(x) + i
            idx.append(i)
            i = i + 1
            found = True
        except ValueError:
            found = False
    return idx


class Navdatabase:
    """
    Navdatabase class definition : command stack & processing class

    Methods:
        Navdatabase()          :  constructor

        findid(txt,lat,lon)    : find a nav closest to lat,lon


    Members:
        wpid                      : list of identifier/short names
        wpname                    : long name
        wptype                    : type of waypoint (yet unused)
        wplat                     : latitude
        wplon                     : longitude
        wpco                      : country code

        apid                      : list of identifier/short names
        apname                    : long name
        aplat                     : latitude
        aplon                     : longitude
        aptype                    : type of airport (1=large, 2=medium, 3=small)
        apmaxrwy                  : max rwy length in meters
        apco                      : country code
        apelev                    : country code


    Created by  : Jacco M. Hoekstra (TU Delft)
    """

    def __init__(self):
        """The navigation database: Contains waypoint, airport, airway, and sector data, but also
        geographical graphics data."""
        # Variables are initialized in reset()
        self.reset()

    def reset(self):
        # print("Loading global navigation database...")
        # wptdata, aptdata, awydata, firdata, codata, rwythresholds = load_navdata()

        nav_data_path = minisky.data(minisky.core.settings.navdata_path)

        wptdata = pd.read_parquet(nav_data_path / "waypoint.parquet")
        aptdata = pd.read_parquet(nav_data_path / "airport.parquet")
        awydata = pd.read_parquet(nav_data_path / "airway.parquet")
        codata = pd.read_parquet(nav_data_path / "country.parquet")

        firdata = json.load(open(nav_data_path / "fir.json"))
        rwythresholds = json.load(open(nav_data_path / "runway_thresholds.json"))

        # Get waypoint data
        self.wpid = wptdata["wpid"].to_list()  # identifier (string)
        self.wplat = wptdata["wplat"].to_list()  # latitude [deg]
        self.wplon = wptdata["wplon"].to_list()  # longitude [deg]
        self.wptype = wptdata["wptype"].to_list()  # type (string)
        self.wpelev = wptdata["wpelev"].to_list()  # elevation [m]
        self.wpvar = wptdata["wpvar"].to_list()  # magn variation [deg]
        self.wpfreq = wptdata["wpfreq"].to_list()  # frequency [kHz/MHz]
        self.wpdesc = wptdata["wpdesc"].to_list()  # description

        # Get airway legs data
        self.awfromwpid = awydata["awfromwpid"].to_list()  # identifier (string)
        self.awfromlat = awydata["awfromlat"].to_list()  # latitude [deg]
        self.awfromlon = awydata["awfromlon"].to_list()  # longitude [deg]
        self.awtowpid = awydata["awtowpid"].to_list()  # identifier (string)
        self.awtolat = awydata["awtolat"].to_list()  # latitude [deg]
        self.awtolon = awydata["awtolon"].to_list()  # longitude [deg]
        self.awid = awydata["awid"].to_list()  # airway identifier (string)
        self.awndir = awydata["awndir"].to_list()  # number of directions (1 or 2)
        self.awlowfl = awydata["awlowfl"].to_list()  # lower flight level (int)
        self.awupfl = awydata["awupfl"].to_list()  # upper flight level (int)

        # Get airpoint data
        self.aptid = aptdata["apid"].to_list()  # 4 char identifier (string)
        self.aptname = aptdata["apname"].to_list()  # full name
        self.aptlat = aptdata["aplat"].to_list()  # latitude [deg]
        self.aptlon = aptdata["aplon"].to_list()  # longitude [deg]
        self.aptmaxrwy = aptdata["apmaxrwy"].to_list()  # max runway length [m]
        self.aptype = aptdata["aptype"].to_list()  # 1=large, 2=medium, 3=small
        self.aptco = aptdata["apco"].to_list()  # two char country code (string)
        self.aptelev = aptdata["apelev"].to_list()  # elevation in meters [m] MSL

        # Get FIR data
        self.fir = firdata["fir"]  # fir name
        self.firlat0 = firdata["firlat0"]  # start lat of a line of border
        self.firlon0 = firdata["firlon0"]  # start lon of a line of border
        self.firlat1 = firdata["firlat1"]  # end lat of a line of border
        self.firlon1 = firdata["firlon1"]  # end lon of a line of border

        # Get country code data
        self.coname = codata["coname"].to_list()  # full name
        self.cocode2 = codata["cocode2"].to_list()  # 2 chars
        self.cocode3 = codata["cocode3"].to_list()  # 3 chars
        self.conr = codata["conr"].to_list()  # country icao number

        self.rwythresholds = rwythresholds

    def defwpt(self, name=None, lat=None, lon=None, wptype=None):
        # Prevent polluting the database: check arguments
        if name == None or name == "":
            return False, "Insufficient arguments"
        elif name.isdigit():
            return False, "Name needs to start with an alphabetical character"

        # DEL command: give info on waypoint (shudl work wit or without lat,lon, may be clicked by accident
        elif (
            not wptype == None
            and (wptype.upper() == "DEL" or wptype.upper() == "DELETE")
        ) or (type(lon) == str and (lon.upper() == "DEL" or lon.upper == "DELETE")):
            return self.delwpt(name)

        # No data: give info on waypoint
        elif lat == None or lon == None:
            reflat, reflon = minisky.scr.getviewctr()
            if self.wpid.count(name.upper()) > 0:
                i = self.getwpidx(name.upper(), reflat, reflon)
                txt = (
                    self.wpid[i] + " : " + str(self.wplat[i]) + "," + str(self.wplon[i])
                )
                if len(self.wptype[i]) > 0:
                    txt = txt + "  " + self.wptype[i]
                return True, txt

            # Waypoint name is free
            else:
                return True, "Waypoint " + name.upper() + " does not yet exist."

        # Still here? So there is data, then we add this waypoint
        self.wpid.append(name.upper())
        self.wplat = np.append(self.wplat, lat)
        self.wplon = np.append(self.wplon, lon)

        if wptype == None:
            self.wptype.append("")
        else:
            self.wptype.append(wptype)

        self.wpelev.append(0.0)  # elevation [m]
        self.wpvar.append(0.0)  # magn variation [deg]
        self.wpfreq.append(0.0)  # frequency [kHz/MHz]
        self.wpdesc.append("Custom waypoint")  # description

        # Update screen info
        minisky.scr.addnavwpt(name.upper(), lat, lon)

        return True, name.upper() + " added to navdb."

    def delwpt(self, name=None):
        """Delete a waypoint"""
        if self.wpid.count(name.upper()) <= 0:
            return False, "Waypoint " + name.upper() + " does not exist."

        idx = (
            len(self.wpid) - self.wpid[::-1].index(name) - 1
        )  # Search from back of list

        del self.wpid[idx]  # wp name

        np.delete(self.wplat, idx)  # wp lat
        np.delete(self.wplon, idx)  # wp lon

        del self.wptype[idx]  # Waypoint type
        del self.wpelev[idx]  # elevation [m]
        del self.wpvar[idx]  # magn variation [deg]
        del self.wpfreq[idx]  # frequency [kHz/MHz]
        del self.wpdesc[idx]  # description

        # Update screen info 9delete necessary there?)
        minisky.scr.removenavwpt(name.upper())

        return True, name.upper() + " deleted from navdb."

    def getwpidx(self, txt, reflat=999999.0, reflon=999999):
        """Get waypoint index to access data"""
        name = txt.upper()
        try:
            i = self.wpid.index(name)
        except:
            return -1

        # if no pos is specified, get first occurence
        if not reflat < 99999.0:
            return i

        # If pos is specified check for more and return closest
        else:
            idx = []
            idx.append(i)
            found = True
            while i < len(self.wpid) - 1 and found:
                try:
                    i = self.wpid.index(name, i + 1)
                    idx.append(i)
                except:
                    found = False
            if len(idx) == 1:
                return idx[0]
            else:
                imin = idx[0]
                dmin = geo.kwikdist(reflat, reflon, self.wplat[imin], self.wplon[imin])
                for i in idx[1:]:
                    d = geo.kwikdist(reflat, reflon, self.wplat[i], self.wplon[i])
                    if d < dmin:
                        imin = i
                        dmin = d
                return imin

    def getwpindices(self, txt, reflat=999999.0, reflon=999999, crit=1852.0):
        """Get waypoint index to access data"""
        name = txt.upper()
        try:
            i = self.wpid.index(name)
        except:
            return [-1]

        # if no pos is specified, get first occurence
        if not reflat < 99999.0:
            return [i]

        # If pos is specified check for more and return closest
        else:
            idx = findall(self.wpid, name)  # find indices of al occurences

            if len(idx) == 1:
                return [idx[0]]
            else:
                imin = idx[0]
                dmin = geo.kwikdist(reflat, reflon, self.wplat[imin], self.wplon[imin])
                for i in idx[1:]:
                    d = geo.kwikdist(reflat, reflon, self.wplat[i], self.wplon[i])
                    if d < dmin:
                        imin = i
                        dmin = d
                # Find co-located
                indices = [imin]
                for i in idx:
                    if i != imin:
                        dist = nm * geo.kwikdist(
                            self.wplat[i],
                            self.wplon[i],
                            self.wplat[imin],
                            self.wplon[imin],
                        )
                        if dist <= crit:
                            indices.append(i)

                return indices

    def getaptidx(self, txt):
        """Get waypoint index to access data"""
        try:
            return self.aptid.index(txt.upper())
        except:
            return -1

    def getinear(self, wlat, wlon, lat, lon):  # lat,lon in degrees
        # t0 = time.clock()
        f = np.cos(np.radians(lat))
        dlat = (wlat - lat + 180.0) % 360.0 - 180.0
        dlon = f * ((wlon - lon + 180.0) % 360.0 - 180.0)
        d2 = dlat * dlat + dlon * dlon
        idx = np.argmin(d2)
        # dt = time.clock()-t0
        # print dt
        return idx

    def getwpinear(self, lat, lon):  # lat,lon in degrees
        """Get closest waypoint index"""
        return self.getinear(self.wplat, self.wplon, lat, lon)

    def getapinear(self, lat, lon):  # lat,lon in degrees
        """Get closest airport index"""
        return self.getinear(self.aptlat, self.aptlon, lat, lon)

    def getinside(self, wlat, wlon, lat0, lat1, lon0, lon1):
        """Get indices inside given box"""
        # t0 = time.clock()
        if lat0 < lat1:
            arr = np.where(
                (wlat > lat0) * (wlat < lat1) * (wlon > lon0) * (wlon < lon1)
            )
        else:
            arr = np.where(
                (wlat > lat1) + (wlat < lat0) * (wlon > lon0) * (wlon < lon1)
            )

        # dt = time.clock()-t0
        # print dt
        return list(arr[0])  # Get indices

    def getwpinside(self, lat0, lat1, lon0, lon1):
        """Get waypoint indices inside box"""
        return self.getinside(self.wplat, self.wplon, lat0, lat1, lon0, lon1)

    def getapinside(self, lat0, lat1, lon0, lon1):
        """Get airport indicex inside box"""
        return self.getinside(self.aptlat, self.aptlon, lat0, lat1, lon0, lon1)

    # returns all runways of given airport
    def listairway(self, airwayid):
        awkey = airwayid.upper()

        airway = []  # identifier of waypoint   0 .. N-1

        # Does this airway exist?
        if self.awid.count(awkey) > 0:
            # Collect leg indices
            i = 0
            found = True
            legs = []  # Alle leg incl. duplicate legs
            left = []  # wps in left column in file
            right = []  # wps in right coumn in file

            idx = findall(self.awid, awkey)
            for i in idx:
                newleg = self.awfromwpid[i] + "-" + self.awtowpid[i]
                if newleg not in legs:
                    legs.append(newleg)
                    left.append(self.awfromwpid[i])
                    right.append(self.awtowpid[i])

            # Not found: return
            if len(legs) == 0:
                return []

            # Count wps to see when we have all segments
            unused = len(left) + len(right)

            while unused > 0 and not left == len(left) * [""]:
                # Find start of a segment
                wps = left + right
                iwps = 0
                while iwps < len(wps) and wps.count(wps[iwps]) > 1:
                    iwps = iwps + 1

                i = iwps % len(left)
                j = int(iwps / len(left))

                # Catch single lost wps
                if j > 1 or iwps > len(wps):
                    break

                # Sort
                wps = [left, right]
                segment = []

                segready = False
                while not segready:
                    # Get leg
                    curwp = wps[j][i]
                    nextwp = wps[1 - j][i]

                    # Update admin of to do wplist
                    unused = unused - 2
                    wps[j][i] = ""
                    wps[1 - j][i] = ""

                    # Add first wp to segment
                    segment.append(curwp)

                    # Find next lef with nextwp
                    if wps[0].count(nextwp) > 0:
                        j = 0
                        i = wps[0].index(nextwp)
                        found = True

                    elif wps[1].count(nextwp) > 0:
                        i = wps[1].index(nextwp)
                        j = 1
                        found = True
                    else:
                        found = False

                    # This segemnt done?
                    segready = (not found) or curwp == "" or nextwp == ""

                # Also add final nextwp of this segment
                segment.append(nextwp)

                # Airway cab have multiple separate segments
                airway.append(segment)

                # Ready for next segment
                left = wps[0]
                right = wps[1]

        return airway  # ,connect

    def listconnections(self, wpid, wplat, wplon):
        # Return list of connecting airway legs
        connect = []

        # Check from-list first
        if wpid in self.awfromwpid:
            idx = findall(self.awfromwpid, wpid)
            for i in idx:
                newitem = [self.awid[i], self.awtowpid[i]]
                if (newitem not in connect) and geo.kwikdist(
                    self.awfromlat[i], self.awfromlon[i], wplat, wplon
                ) < 10.0:
                    connect.append(newitem)

        # Check to-list nextt
        if wpid in self.awtowpid:
            idx = findall(self.awtowpid, wpid)
            for i in idx:
                newitem = [self.awid[i], self.awfromwpid[i]]
                if (newitem not in connect) and geo.kwikdist(
                    self.awtolat[i], self.awtolon[i], wplat, wplon
                ) < 10.0:
                    connect.append(newitem)

        return connect  # return list of [awid,wpid]
