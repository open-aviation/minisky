"""X-Plane-derived navigation datasets for MiniSky."""

from __future__ import annotations

import json
from importlib.resources import files
from importlib.resources.abc import Traversable

import numpy as np
import polars as pl
from minisky import (
    AirportData,
    AirwayData,
    CountryData,
    FirBoundary,
    FirData,
    NavData,
    RunwayThreshold,
    RunwayThresholdData,
    WaypointData,
)
from minisky import quantities as q

__all__ = (
    "load",
    "load_airports",
    "load_airways",
    "load_countries",
    "load_firs",
    "load_runway_thresholds",
    "load_waypoints",
)


def _data_path() -> Traversable:
    return files("minisky_xplane_navdata").joinpath("data")


def load_waypoints() -> WaypointData:
    data = (
        pl.scan_parquet(str(_data_path().joinpath("waypoint.parquet")))
        .select(
            pl.col("wpid").cast(pl.String).alias("identifiers"),
            pl.col("wplat").cast(pl.Float64).alias("latitudes"),
            pl.col("wplon").cast(pl.Float64).alias("longitudes"),
            pl.col("wptype").cast(pl.String).alias("categories"),
            pl.col("wpelev").cast(pl.Float64).alias("elevations"),
            pl.col("wpvar").cast(pl.Float64).alias("magnetic_variations"),
            (
                pl.when(pl.col("wptype").is_in(("VOR", "DME", "TACAN")))
                .then(q.mhz_to_hz(pl.col("wpfreq").cast(pl.Float64)))
                .otherwise(q.khz_to_hz(pl.col("wpfreq").cast(pl.Float64)))
                .alias("frequencies")
            ),
            pl.col("wpdesc").cast(pl.String).alias("descriptions"),
        )
        .collect()
    )
    return WaypointData(
        identifiers=np.asarray(data["identifiers"]),
        latitudes=np.asarray(data["latitudes"]),
        longitudes=np.asarray(data["longitudes"]),
        categories=np.asarray(data["categories"]),
        elevations=np.asarray(data["elevations"]),
        magnetic_variations=np.asarray(data["magnetic_variations"]),
        frequencies=np.asarray(data["frequencies"]),
        descriptions=np.asarray(data["descriptions"]),
    )


def load_airports() -> AirportData:
    data = (
        pl.scan_parquet(str(_data_path().joinpath("airport.parquet")))
        .select(
            pl.col("apid").cast(pl.String).alias("identifiers"),
            pl.col("apname").cast(pl.String).alias("names"),
            pl.col("aplat").cast(pl.Float64).alias("latitudes"),
            pl.col("aplon").cast(pl.Float64).alias("longitudes"),
            pl.col("apmaxrwy").cast(pl.Float64).alias("max_runway_lengths"),
            pl.col("aptype").cast(pl.Int64).alias("sizes"),
            pl.col("apco").cast(pl.String).alias("countries"),
            pl.col("apelev").cast(pl.Float64).alias("elevations"),
        )
        .collect()
    )
    return AirportData(
        identifiers=np.asarray(data["identifiers"]),
        names=np.asarray(data["names"]),
        latitudes=np.asarray(data["latitudes"]),
        longitudes=np.asarray(data["longitudes"]),
        max_runway_lengths=np.asarray(data["max_runway_lengths"]),
        sizes=np.asarray(data["sizes"]),
        countries=np.asarray(data["countries"]),
        elevations=np.asarray(data["elevations"]),
    )


def load_airways() -> AirwayData:
    data = (
        pl.scan_parquet(str(_data_path().joinpath("airway.parquet")))
        .select(
            pl.col("awid").cast(pl.String).alias("identifiers"),
            pl.col("awfromwpid").cast(pl.String).alias("from_waypoints"),
            pl.col("awfromlat").cast(pl.Float64).alias("from_latitudes"),
            pl.col("awfromlon").cast(pl.Float64).alias("from_longitudes"),
            pl.col("awtowpid").cast(pl.String).alias("to_waypoints"),
            pl.col("awtolat").cast(pl.Float64).alias("to_latitudes"),
            pl.col("awtolon").cast(pl.Float64).alias("to_longitudes"),
            pl.col("awndir").cast(pl.Int64).alias("directions"),
            q.ft_to_m(pl.col("awlowfl").cast(pl.Float64) * 100.0).alias("lower_altitudes"),
            q.ft_to_m(pl.col("awupfl").cast(pl.Float64) * 100.0).alias("upper_altitudes"),
        )
        .collect()
    )
    return AirwayData(
        identifiers=np.asarray(data["identifiers"]),
        from_waypoints=np.asarray(data["from_waypoints"]),
        from_latitudes=np.asarray(data["from_latitudes"]),
        from_longitudes=np.asarray(data["from_longitudes"]),
        to_waypoints=np.asarray(data["to_waypoints"]),
        to_latitudes=np.asarray(data["to_latitudes"]),
        to_longitudes=np.asarray(data["to_longitudes"]),
        directions=np.asarray(data["directions"]),
        lower_altitudes=np.asarray(data["lower_altitudes"]),
        upper_altitudes=np.asarray(data["upper_altitudes"]),
    )


def load_firs() -> FirData:
    with _data_path().joinpath("fir.json").open() as file:
        data = json.load(file)
    return FirData(
        boundaries=tuple(
            FirBoundary(
                identifier,
                np.asarray(latitudes, dtype=float),
                np.asarray(longitudes, dtype=float),
            )
            for identifier, latitudes, longitudes in data["fir"]
        ),
        segment_start_latitudes=np.asarray(data["firlat0"], dtype=float),
        segment_start_longitudes=np.asarray(data["firlon0"], dtype=float),
        segment_end_latitudes=np.asarray(data["firlat1"], dtype=float),
        segment_end_longitudes=np.asarray(data["firlon1"], dtype=float),
    )


def load_countries() -> CountryData:
    data = (
        pl.scan_parquet(str(_data_path().joinpath("country.parquet")))
        .select(
            pl.col("coname").cast(pl.String).alias("names"),
            pl.col("cocode2").cast(pl.String).alias("codes2"),
            pl.col("cocode3").cast(pl.String).alias("codes3"),
            pl.col("conr").cast(pl.Int64).alias("numbers"),
        )
        .collect()
    )
    return CountryData(
        names=np.asarray(data["names"]),
        codes2=np.asarray(data["codes2"]),
        codes3=np.asarray(data["codes3"]),
        numbers=np.asarray(data["numbers"]),
    )


def load_runway_thresholds() -> RunwayThresholdData:
    with _data_path().joinpath("runway_thresholds.json").open() as file:
        data = json.load(file)
    return {
        airport: {
            runway: RunwayThreshold(float(values[0]), float(values[1]), float(values[2]))
            for runway, values in runways.items()
        }
        for airport, runways in data.items()
    }


def load() -> NavData:
    return NavData(
        waypoints=load_waypoints(),
        airports=load_airports(),
        airways=load_airways(),
        firs=load_firs(),
        countries=load_countries(),
        runway_thresholds=load_runway_thresholds(),
    )
