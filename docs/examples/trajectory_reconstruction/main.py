from __future__ import annotations

from collections.abc import Iterable, Iterator
from dataclasses import dataclass
from datetime import datetime, timedelta
from itertools import pairwise
from pathlib import Path
from typing import NamedTuple, TypeAlias

import matplotlib.dates as mdates
import matplotlib.pyplot as plt
import numpy as np
import polars as pl
from matplotlib.axes import Axes
from matplotlib.figure import Figure
from matplotlib.lines import Line2D
from minisky import CoordinateWaypoint, MiniSky, MiniSkyConfig, aero, geo
from minisky import quantities as q
from minisky.types import CasMps, Mach, StdPressureAltM

RADIUS_EARTH: q.DistanceM[float] = 6_371_000.0


#
# lateral, vertical and speed intent
#


# --8<-- [start:lateral0]
@dataclass(frozen=True, slots=True)
class Waypoint:
    lat: q.LatitudeDeg[float]
    lon: q.LongitudeDeg[float]


# --8<-- [end:lateral0]
# --8<-- [start:vertical0]
@dataclass(frozen=True, slots=True)
class AltitudeSelection:
    time: q.DurationS[float]
    alt: q.PressureAltitudeM[float]


# --8<-- [end:vertical0]
# --8<-- [start:speed0]


@dataclass(frozen=True, slots=True)
class CalibratedAirspeedSelection:
    time: q.DurationS[float]
    cas: q.CalibratedAirspeedMps[float]


@dataclass(frozen=True, slots=True)
class MachSelection:
    time: q.DurationS[float]
    mach: q.MachNumber[float]


SpeedInstruction: TypeAlias = CalibratedAirspeedSelection | MachSelection
# --8<-- [end:speed0]


#
# infer lateral intent with douglas-peucker
#


class Segment(NamedTuple):
    start: int
    end: int


def douglas_peucker_indices(
    lat: q.LatitudeDeg, lon: q.LongitudeDeg, tolerance_m: q.DistanceM[float]
) -> np.ndarray:
    """Return retained route points in source order."""
    lat_r: q.AngleRad = np.radians(np.asarray(lat))
    lon_r: q.AngleRad = np.unwrap(np.radians(np.asarray(lon)))
    lat_ref: q.AngleRad[float] = float(np.mean(lat_r))
    lon_ref: q.AngleRad[float] = float(lon_r[0])

    points: q.DistanceM = np.empty((len(lat_r), 2))
    points[:, 0] = RADIUS_EARTH * (lon_r - lon_ref) * np.cos(lat_ref)
    points[:, 1] = RADIUS_EARTH * lat_r
    keep = np.zeros(len(points), dtype=bool)
    keep[[0, -1]] = True
    segments = [Segment(0, len(points) - 1)]

    while segments:
        start, end = segments.pop()
        if end - start < 2:
            continue

        segment = points[end] - points[start]
        offsets = points[start + 1 : end] - points[start]
        length_squared = float(segment @ segment)
        if length_squared == 0.0:
            distances: q.DistanceM = np.hypot(offsets[:, 0], offsets[:, 1])
        else:
            projection = np.clip(offsets @ segment / length_squared, 0.0, 1.0)
            residual = offsets - projection[:, np.newaxis] * segment
            distances = np.hypot(residual[:, 0], residual[:, 1])
        relative = int(distances.argmax())
        if distances[relative] <= tolerance_m:
            continue

        index = start + relative + 1
        keep[index] = True
        segments.extend((Segment(start, index), Segment(index, end)))

    return keep.nonzero()[0]


#
# infer selected altitude from bds 4,0
#


class SettledStep(NamedTuple):
    time: q.DurationS[float]
    value: float


def settled_steps(time: q.DurationS, values, settle: q.DurationS[float]) -> Iterator[SettledStep]:
    """Yield discrete values only after they remain unchanged for `settle`."""
    starts = np.flatnonzero(np.concatenate(([True], values[1:] != values[:-1])))
    ends = np.concatenate((starts[1:], [len(values)]))
    for start, end in zip(starts, ends, strict=True):
        if time[end - 1] - time[start] >= settle:
            yield SettledStep(float(time[start]), float(values[start]))


def infer_altitude_instructions(
    raw: pl.LazyFrame,
    source: pl.DataFrame,
    settle: q.DurationS[float],
) -> Iterator[AltitudeSelection]:
    start: datetime = source["timestamp"][0]
    stop: datetime = source["timestamp"][-1]
    selected = (
        raw.filter(pl.col("timestamp").is_between(start, stop))
        .select("epoch_s", "selected_alt")
        .drop_nulls()
        .collect()
    )
    selected_time: q.DurationS = selected["epoch_s"].to_numpy() - start.timestamp()

    # --8<-- [start:vertical1]
    for step in settled_steps(selected_time, selected["selected_alt"].to_numpy(), settle):
        yield AltitudeSelection(time=step.time, alt=step.value)
    # --8<-- [end:vertical1]


#
# infer speed schedule from bds 6,0
# WRAP fits piecewise CAS/Mach profiles and validates the crossover
# but we keep it simple for now.
# note that minisky does not support controlling the speed rate (it uses the aircraft performance model's axmax)
#


class Plateau(NamedTuple):
    start: int
    end: int
    value: float


def stable_plateaus(
    time: q.DurationS, values, tolerance: float, min_duration: q.DurationS[float]
) -> Iterator[Plateau]:
    """Yield long, nearly flat regimes represented by their median value."""
    start = 0
    while start < len(values):
        end = int(np.searchsorted(time, time[start] + min_duration, side="left"))
        if end >= len(values):
            return
        if float(np.ptp(values[start : end + 1])) > tolerance:
            start += 1
            continue

        while end + 1 < len(values) and float(np.ptp(values[start : end + 2])) <= tolerance:
            end += 1
        yield Plateau(start, end, float(np.median(values[start : end + 1])))
        start = end + 1


def tail_median(time: q.DurationS, values, duration: q.DurationS[float]) -> float:
    """Return the median over the final `duration` seconds."""
    start = int(np.searchsorted(time, time[-1] - duration, side="left"))
    return float(np.median(values[start:]))


def merge_plateaus(
    time: q.DurationS,
    values,
    plateaus: Iterable[Plateau],
    value_tolerance: float,
    max_gap: q.DurationS[float],
) -> Iterator[Plateau]:
    iterator = iter(plateaus)
    current = next(iterator, None)
    if current is None:
        return

    for plateau in iterator:
        gap = time[plateau.start] - time[current.end]
        if gap <= max_gap and abs(plateau.value - current.value) <= value_tolerance:
            current = Plateau(
                current.start,
                plateau.end,
                float(np.median(values[current.start : plateau.end + 1])),
            )
            continue
        yield current
        current = plateau
    yield current


def infer_speed_instructions(
    source: pl.DataFrame,
    plateau_tolerance: q.CalibratedAirspeedMps[float],
    plateau_duration: q.DurationS[float],
    merge_tolerance: q.CalibratedAirspeedMps[float],
    merge_gap: q.DurationS[float],
    mach_tail_duration: q.DurationS[float],
) -> Iterator[SpeedInstruction]:
    time: q.DurationS = source["time"].to_numpy()
    altitude: q.PressureAltitudeM = source["alt"].to_numpy()
    cas: q.CalibratedAirspeedMps = source["cas"].to_numpy()
    # --8<-- [start:speed1]
    mach = tail_median(time, source["mach"].to_numpy(), mach_tail_duration)
    regimes = merge_plateaus(
        time,
        cas,
        stable_plateaus(time, cas, plateau_tolerance, plateau_duration),
        merge_tolerance,
        merge_gap,
    )

    for previous, regime in pairwise(regimes):
        yield CalibratedAirspeedSelection(
            time=float(time[previous.end + 1]),
            cas=regime.value,
        )

        crossover_altitude = aero.crossoveralt(regime.value, mach)
        crossover = np.flatnonzero(altitude >= crossover_altitude)
        if len(crossover) == 0:
            continue
        crossover_index = int(crossover[0])
        if regime.start <= crossover_index <= regime.end:
            yield MachSelection(time=float(time[crossover_index]), mach=mach)
            return
    # --8<-- [end:speed1]


#
# minisky reconstruction
#


@dataclass(frozen=True, slots=True)
class InitialState:
    callsign: str
    ac_type: str
    lat: q.LatitudeDeg[float]
    lon: q.LongitudeDeg[float]
    hdg: q.TrueHeadingDegrees[float]
    alt: q.PressureAltitudeM[float]
    cas: q.CalibratedAirspeedMps[float]
    vs: q.VerticalRateMps[float]


# --8<-- [start:plan]
Instruction: TypeAlias = AltitudeSelection | SpeedInstruction


@dataclass(frozen=True, slots=True)
class TrajectoryPlan:
    initial: InitialState
    waypoints: tuple[Waypoint, ...]
    instructions: tuple[Instruction, ...]
    duration: q.DurationS[float]
    timestep: q.DurationS[float] = 1.0


class Reconstruction(NamedTuple):
    time: q.DurationS[float]
    lat: q.LatitudeDeg[float]
    lon: q.LongitudeDeg[float]
    alt: q.PressureAltitudeM[float]
    cas: q.CalibratedAirspeedMps[float]
    vs: q.VerticalRateMps[float]


# --8<-- [end:plan]


def reconstruct(
    plan: TrajectoryPlan, *, config: MiniSkyConfig | None = None
) -> Iterator[Reconstruction]:
    """Execute a plan and yield every simulation boundary."""
    # --8<-- [start:replay]
    with MiniSky(config) as runtime:
        runtime.simulation.simdt = plan.timestep
        initial = plan.initial
        runtime.traffic.cre(
            initial.callsign.upper(),
            actype=initial.ac_type,
            lat=initial.lat,
            lon=initial.lon,
            hdg=initial.hdg,
            alt=StdPressureAltM(initial.alt),
            airspeed=CasMps(initial.cas),
        ).unwrap()
        # NOTE: WRAP's statistical A320 maximum (~317 kt) is inconsistent with
        # OpenAP's VMO of 350kt.
        # See c93774a: the speed envelope is evaluated at the current altitude
        # so we force 350kt for now.
        runtime.traffic.perf.vmaxer[0] = q.kt_to_mps(350.0)

        # cre starts from level flight and doesn't allow us to specify initial VS
        runtime.traffic.vs[0] = initial.vs

        for waypoint in plan.waypoints:
            runtime.route_commands.append_waypoint(
                0, CoordinateWaypoint(waypoint.lat, waypoint.lon)
            ).unwrap()
        if plan.waypoints:
            route = runtime.traffic.ap.route[0]
            runtime.route_commands.direct(0, route.wpname[0]).unwrap()

        def record() -> Reconstruction:
            traffic = runtime.traffic
            return Reconstruction(
                time=runtime.simulation.simt,
                lat=float(traffic.lat[0]),
                lon=float(traffic.lon[0]),
                alt=float(traffic.alt[0]),
                cas=float(traffic.cas[0]),
                vs=float(traffic.vs[0]),
            )

        aircraft = np.asarray([0], dtype=int)
        instruction_idx = 0
        yield record()
        while runtime.simulation.simt < plan.duration:
            while (
                instruction_idx < len(plan.instructions)
                and plan.instructions[instruction_idx].time <= runtime.simulation.simt
            ):
                match plan.instructions[instruction_idx]:
                    case AltitudeSelection(alt=selected_alt):
                        # NOTE: the default vertical rate during climb is 1500fpm.
                        runtime.traffic.ap.selaltcmd(
                            aircraft, StdPressureAltM(selected_alt)
                        ).unwrap()
                    case CalibratedAirspeedSelection(cas=selected_cas):
                        runtime.traffic.ap.select_airspeed(aircraft, CasMps(selected_cas)).unwrap()
                    case MachSelection(mach=selected_mach):
                        runtime.traffic.ap.select_airspeed(aircraft, Mach(selected_mach)).unwrap()
                instruction_idx += 1

            next_instruction = (
                plan.instructions[instruction_idx].time
                if instruction_idx < len(plan.instructions)
                else plan.duration
            )
            runtime.simulation.simdt = (
                min(
                    runtime.simulation.simt + plan.timestep,
                    next_instruction,
                    plan.duration,
                )
                - runtime.simulation.simt
            )
            if not runtime.simulation.step():
                raise RuntimeError("simulation step is waiting for an asynchronous command")
            yield record()
    # --8<-- [end:replay]


#
# data loading
#


RS1090_SCHEMA = {
    "timestamp": pl.Float64,
    "latitude": pl.Float64,
    "longitude": pl.Float64,
    "altitude": pl.Float64,
    "IAS": pl.Float64,
    "Mach": pl.Float64,
    "vertical_rate": pl.Float64,
    "vrate_barometric": pl.Float64,
    "selected_mcp": pl.Float64,
}
SOURCE_FIELDS = ("latitude", "longitude", "alt", "cas", "mach", "vs", "baro_vs")


def sample_path(
    output: Path,
    *,
    filename: str = "full_flight_short.jsonl",
    sample_url: str = "https://raw.githubusercontent.com/xoolive/traffic/aee9ba062051b6b5619b5c486be2b6e24f66a1fb/src/traffic/data/samples/rs1090/full_flight_short.jsonl",
) -> Path:
    sample_cache = output / filename
    if not sample_cache.exists():
        output.mkdir(parents=True, exist_ok=True)
        import httpx2

        response = httpx2.get(sample_url)
        response.raise_for_status()
        sample_cache.write_bytes(response.content)
    return sample_cache


def scan_sample(path: Path) -> pl.LazyFrame:
    # --8<-- [start:load-data]
    return (
        pl.scan_ndjson(path, schema=RS1090_SCHEMA)
        .select(
            pl.col("timestamp").alias("epoch_s"),
            (
                (pl.col("timestamp") * 1_000_000)
                .cast(pl.Int64)
                .cast(pl.Datetime("us", "UTC"))
                .alias("timestamp")
            ),
            "latitude",
            "longitude",
            q.ft_to_m(pl.col("altitude")).alias("alt"),
            q.kt_to_mps(pl.col("IAS")).alias("cas"),
            pl.col("Mach").alias("mach"),
            q.fpm_to_mps(pl.col("vertical_rate")).alias("vs"),
            q.fpm_to_mps(pl.col("vrate_barometric")).alias("baro_vs"),
            q.ft_to_m(pl.col("selected_mcp")).alias("selected_alt"),
        )
        .sort("timestamp")
    )
    # --8<-- [end:load-data]


def load_source(raw: pl.LazyFrame) -> pl.DataFrame:
    start, stop = (
        raw.select(
            pl.col("timestamp").min().alias("start"),
            pl.col("timestamp").max().alias("stop"),
        )
        .collect()
        .row(0)
    )
    start = start.replace(microsecond=0)
    stop = stop.replace(microsecond=0)

    grid = pl.LazyFrame(
        {"timestamp": pl.datetime_range(start, stop, interval="1s", time_zone="UTC", eager=True)}
    )
    samples = raw.group_by_dynamic("timestamp", every="1s").agg(
        pl.col(field).drop_nulls().first().alias(field) for field in SOURCE_FIELDS
    )
    source = (
        grid.join(samples, on="timestamp", how="left")
        .sort("timestamp")
        .with_columns(pl.col(SOURCE_FIELDS).interpolate())
        .drop_nulls(SOURCE_FIELDS)
        .collect()
    )
    peak_index = source.select(pl.col("alt").arg_max()).item()
    source = source.head(peak_index + 1)
    start_us = source["timestamp"].cast(pl.Int64)[0]
    return source.with_columns(
        ((pl.col("timestamp").cast(pl.Int64) - start_us) / 1_000_000).alias("time")
    )


#
# plotting and evaluation
#

LABEL_TRAJ_ACTUAL = "raw measurements"
LABEL_TRAJ_RECONSTRUCTED = "minisky reconstructed trajectory"
LABEL_ALT = "altitude instruction (from BDS 4,0)"
LABEL_CAS = "CAS instruction (inferred from BDS 6,0)"
LABEL_MACH = "Mach instruction (inferred from BDS 6,0)"


class Metrics(NamedTuple):
    sspd: q.DistanceM[float]
    dtw: q.DistanceM[float]
    alt_mae: q.VerticalDistanceFt[float]
    cas_mae: q.CalibratedAirspeedKt[float]
    mach_mae: q.MachNumber[float]


class PlotAxes(NamedTuple):
    figure: Figure
    map: Axes
    altitude: Axes
    vertical_rate: Axes
    cas: Axes
    mach: Axes


def _profile_measurements(raw: pl.DataFrame, expression: pl.Expr) -> tuple[np.ndarray, np.ndarray]:
    data = raw.select("timestamp", expression.alias("value")).drop_nulls()
    return data["timestamp"].to_numpy(), data["value"].to_numpy()


def plot_measurements(raw: pl.DataFrame, callsign: str) -> PlotAxes:
    figure = plt.figure(figsize=(8, 5))
    body = figure.add_gridspec(
        1,
        2,
        width_ratios=(1.25, 4.4),
        wspace=0.16,
        left=0.035,
        right=0.99,
        top=0.95,
        bottom=0.055,
    )
    profiles = body[1].subgridspec(4, 1, hspace=0.03)

    axes = PlotAxes(
        figure=figure,
        map=figure.add_subplot(body[0]),
        altitude=figure.add_subplot(profiles[0]),
        vertical_rate=figure.add_subplot(profiles[1]),
        cas=figure.add_subplot(profiles[2]),
        mach=figure.add_subplot(profiles[3]),
    )
    axes.vertical_rate.sharex(axes.altitude)
    axes.cas.sharex(axes.altitude)
    axes.mach.sharex(axes.altitude)

    position = raw.select("longitude", "latitude").drop_nulls()
    axes.map.scatter(
        position["longitude"],
        position["latitude"],
        s=1,
        linewidths=0,
        alpha=0.55,
    )
    mean_lat = raw.select(pl.col("latitude").mean()).item()
    axes.map.set_aspect(1 / np.cos(np.radians(mean_lat)))
    axes.map.margins(0.08)
    axes.map.set_xlim(axes.map.get_xlim())
    axes.map.set_ylim(axes.map.get_ylim())  # type: ignore
    axes.map.set_xlabel("longitude [deg]")
    axes.map.set_ylabel("latitude\n[deg]", rotation=0, ha="right", va="center", labelpad=8)
    axes.map.tick_params(labelsize=8)
    axes.map.grid(alpha=0.12, linewidth=0.5)
    axes.map.spines[["top", "right"]].set_visible(False)

    profile_data = (
        (axes.altitude, q.m_to_ft(pl.col("alt")), "altitude\n[ft]"),
        (axes.vertical_rate, q.mps_to_fpm(pl.col("vs")), "vertical rate\n[ft/min]"),
        (axes.cas, q.mps_to_kt(pl.col("cas")), "CAS\n[kt]"),
        (axes.mach, pl.col("mach"), "Mach"),
    )
    for axis, expression, ylabel in profile_data:
        timestamp, values = _profile_measurements(raw, expression)
        axis.scatter(timestamp, values, s=2, linewidths=0, alpha=0.55)
        axis.set_ylabel(ylabel, rotation=0, ha="right", va="center", labelpad=8)
        axis.grid(alpha=0.12, linewidth=0.5)
        axis.spines[["top", "right"]].set_visible(False)
        axis.tick_params(labelsize=8)
        axis.margins(y=0.08)
        axis.set_ylim(axis.get_ylim())  # type: ignore

    axes.altitude.tick_params(labelbottom=False)
    axes.vertical_rate.tick_params(labelbottom=False)
    axes.cas.tick_params(labelbottom=False)
    axes.mach.xaxis.set_major_locator(mdates.MinuteLocator(interval=5))
    axes.mach.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M"))
    axes.mach.set_xlabel("time [UTC]")

    figure.suptitle(callsign, x=0.01, y=0.985, ha="left", fontsize=14)
    return axes


def add_reconstruction_(
    axes: PlotAxes,
    reconstructed: pl.DataFrame,
    plan: TrajectoryPlan,
    metrics: Metrics,
    source_start: datetime,
) -> None:
    colors = {
        LABEL_ALT: "#e45756",
        LABEL_CAS: "#72b7b2",
        LABEL_MACH: "#54a24b",
    }

    reconstructed_lat = reconstructed["latitude"].to_numpy()
    reconstructed_lon = reconstructed["longitude"].to_numpy()
    waypoint_indices: list[int] = []
    search_start = 0
    for waypoint in plan.waypoints:
        error = np.square(reconstructed_lat[search_start:] - waypoint.lat) + np.square(
            reconstructed_lon[search_start:] - waypoint.lon
        )
        index = search_start + int(error.argmin())
        waypoint_indices.append(index)
        search_start = index

    waypoint_time: q.DurationS = reconstructed["time"].to_numpy()[waypoint_indices]
    event_time = sorted(
        {
            *(float(value) for value in waypoint_time),
            *(instruction.time for instruction in plan.instructions),
        }
    )
    event_idx = {event_time: index for index, event_time in enumerate(event_time, start=1)}

    altitude_instructions = [
        instruction
        for instruction in plan.instructions
        if isinstance(instruction, AltitudeSelection)
    ]
    speed_instructions = [
        instruction
        for instruction in plan.instructions
        if isinstance(instruction, (CalibratedAirspeedSelection, MachSelection))
    ]
    instruction_end: dict[Instruction, q.DurationS[float]] = {}
    for instructions in (altitude_instructions, speed_instructions):
        for index, instruction in enumerate(instructions):
            instruction_end[instruction] = (
                instructions[index + 1].time if index + 1 < len(instructions) else plan.duration
            )

    axes.map.plot(
        reconstructed["longitude"],
        reconstructed["latitude"],
        color="C1",
        linewidth=1.4,
        linestyle="--",
    )
    for waypoint, time in zip(plan.waypoints, waypoint_time, strict=True):
        axes.map.scatter(
            waypoint.lon,
            waypoint.lat,
            s=42,
            facecolors="none",
            edgecolors="black",
            linewidths=1.1,
            zorder=3,
        )
        axes.map.annotate(
            str(event_idx[float(time)]),
            (waypoint.lon, waypoint.lat),
            xytext=(-2, 2),
            textcoords="offset points",
            ha="right",
            va="bottom",
            fontsize=7,
        )

    profile_data = (
        (axes.altitude, q.m_to_ft(pl.col("alt"))),
        (axes.vertical_rate, q.mps_to_fpm(pl.col("vs"))),
        (axes.cas, q.mps_to_kt(pl.col("cas"))),
        (axes.mach, pl.col("mach")),
    )
    for axis, expression in profile_data:
        timestamp, values = _profile_measurements(reconstructed, expression)
        axis.plot(timestamp, values, color="C1", linewidth=1.3, linestyle="--")

    profile_axes = (axes.altitude, axes.vertical_rate, axes.cas, axes.mach)
    for time in (float(value) for value in waypoint_time):
        timestamp_num = mdates.date2num(source_start + timedelta(seconds=time))
        for axis in profile_axes:
            axis.axvline(timestamp_num, color="black", alpha=0.15, linewidth=0.6)
        axes.altitude.annotate(
            str(event_idx[time]),
            (timestamp_num, 1),
            xycoords=("data", "axes fraction"),  # type: ignore
            xytext=(-1, -1),
            textcoords="offset points",
            ha="right",
            va="top",
            fontsize=7,
        )

    for instruction in plan.instructions:
        index = event_idx[instruction.time]
        start_num = mdates.date2num(source_start + timedelta(seconds=instruction.time))
        end_num = mdates.date2num(source_start + timedelta(seconds=instruction_end[instruction]))
        match instruction:
            case AltitudeSelection(alt=altitude):
                axis = axes.altitude
                color = colors[LABEL_ALT]
                value = float(q.m_to_ft(altitude))
                label = f"{index}: {value:.0f} ft"
            case CalibratedAirspeedSelection(cas=cas):
                axis = axes.cas
                color = colors[LABEL_CAS]
                value = float(q.mps_to_kt(cas))
                label = f"{index}: {value:.0f} kt"
            case MachSelection(mach=mach):
                axis = axes.mach
                color = colors[LABEL_MACH]
                value = mach
                label = f"{index}: Mach {value:.3f}"
        axis.plot([start_num, end_num], [value, value], color=color, linewidth=1.7, alpha=0.8)
        axis.scatter(start_num, value, color=color, s=14, zorder=3)
        axis.annotate(
            label,
            (start_num, value),
            xytext=(1, 1),
            textcoords="offset points",
            ha="left",
            va="bottom",
            color=color,
            fontsize=7,
        )

    reconstructed_label = (
        f"{LABEL_TRAJ_RECONSTRUCTED}\n"
        f"SSPD {metrics.sspd:.0f} m, DTW {metrics.dtw:.0f} m,\n"
        f"altitude MAE {metrics.alt_mae:.0f} ft, CAS MAE {metrics.cas_mae:.0f} kt,\n"
        f"Mach MAE {metrics.mach_mae:.3f}"
    )
    axes.figure.legend(
        handles=[
            Line2D([], [], color="C0", marker=".", linestyle="none"),
            Line2D([], [], color="C1", linewidth=1.4, linestyle="--"),
            Line2D([], [], marker="o", markerfacecolor="none", linestyle="none"),
            Line2D([], [], color=colors[LABEL_ALT], marker="o", markersize=2.5),
            Line2D([], [], color=colors[LABEL_CAS], marker="o", markersize=2.5),
            Line2D([], [], color=colors[LABEL_MACH], marker="o", markersize=2.5),
        ],
        labels=[
            LABEL_TRAJ_ACTUAL,
            reconstructed_label,
            "waypoints (from douglas-peucker)",
            LABEL_ALT,
            LABEL_CAS,
            LABEL_MACH,
        ],
        loc="upper center",
        bbox_to_anchor=(0.5, -0.045),
        ncols=2,
        frameon=False,
        fontsize=7.5,
        handlelength=2.2,
        handletextpad=0.6,
        columnspacing=1.5,
        labelspacing=0.15,
        borderaxespad=0,
    )


def main(
    *,
    lateral_tolerance: q.DistanceM[float] = 500.0,
    altitude_settle: q.DurationS[float] = 5.0,
    speed_plateau_tolerance: q.CalibratedAirspeedKt[float] = 4.0,
    speed_plateau_duration: q.DurationS[float] = 20.0,
    speed_merge_tolerance: q.CalibratedAirspeedKt[float] = 7.0,
    speed_merge_gap: q.DurationS[float] = 45.0,
    mach_tail_duration: q.DurationS[float] = 180.0,
    output: Path = Path(__file__).parent,
) -> None:
    import traj_dist_rs

    plt.rcParams["font.family"] = "Inter"

    raw = scan_sample(sample_path(output=output))
    source = load_source(raw)
    source_start: datetime = source["timestamp"][0]
    source_stop: datetime = source["timestamp"][-1]
    measurements = raw.filter(pl.col("timestamp").is_between(source_start, source_stop)).collect()

    t: q.DurationS = source["time"].to_numpy()
    lat: q.LatitudeDeg[np.ndarray] = source["latitude"].to_numpy()
    lon: q.LongitudeDeg[np.ndarray] = source["longitude"].to_numpy()
    alt: q.PressureAltitudeM[np.ndarray] = source["alt"].to_numpy()
    cas: q.CalibratedAirspeedMps[np.ndarray] = source["cas"].to_numpy()
    vs_initial: q.VerticalRateMps[float] = source["baro_vs"][0]
    instructions = tuple(
        sorted(
            (
                *infer_altitude_instructions(raw, source, altitude_settle),
                *infer_speed_instructions(
                    source,
                    plateau_tolerance=q.kt_to_mps(speed_plateau_tolerance),
                    plateau_duration=speed_plateau_duration,
                    merge_tolerance=q.kt_to_mps(speed_merge_tolerance),
                    merge_gap=speed_merge_gap,
                    mach_tail_duration=mach_tail_duration,
                ),
            ),
            key=lambda instruction: instruction.time,
        )
    )

    print("commands")
    for instruction in instructions:
        timestamp = source_start + timedelta(seconds=instruction.time)
        if isinstance(instruction, AltitudeSelection):
            command = f"ALT {float(q.m_to_ft(instruction.alt)):.0f} ft"
        elif isinstance(instruction, CalibratedAirspeedSelection):
            command = f"CAS {float(q.mps_to_kt(instruction.cas)):.0f} kt"
        else:
            command = f"MACH {instruction.mach:.3f}"
        print(f"{timestamp:%H:%M:%S}Z {command}")

    # --8<-- [start:lateral1]
    retained = douglas_peucker_indices(lat, lon, lateral_tolerance)
    waypoints = tuple(
        Waypoint(lat=float(lat[index]), lon=float(lon[index])) for index in retained[1:]
    )
    # --8<-- [end:lateral1]

    moving = next(
        (index for index in range(1, len(lat)) if lat[index] != lat[0] or lon[index] != lon[0]),
        1,
    )
    heading: q.TrueHeadingDegrees[float] = float(
        geo.qdrdist(float(lat[0]), float(lon[0]), float(lat[moving]), float(lon[moving]))[0]
    )
    plan = TrajectoryPlan(
        initial=InitialState(
            callsign="AFR34ZG",
            ac_type="A320",
            lat=float(lat[0]),
            lon=float(lon[0]),
            hdg=heading % 360.0,
            alt=float(alt[0]),
            cas=float(cas[0]),
            vs=vs_initial,
        ),
        waypoints=waypoints,
        instructions=instructions,
        duration=float(t[-1]),
    )

    output.mkdir(parents=True, exist_ok=True)
    axes = plot_measurements(measurements, plan.initial.callsign)
    axes.figure.savefig(str(output / "measurements.png"), dpi=300, bbox_inches="tight")

    reconstructed = pl.DataFrame(reconstruct(plan), schema=Reconstruction._fields, orient="row")
    reconstructed = reconstructed.with_columns(
        (
            pl.lit(source_start)
            + pl.duration(microseconds=(pl.col("time") * 1_000_000).round().cast(pl.Int64))
        ).alias("timestamp"),
        pl.Series(
            "mach",
            aero.vcas2mach(reconstructed["cas"].to_numpy(), reconstructed["alt"].to_numpy()),
        ),
    ).rename({"lat": "latitude", "lon": "longitude"})

    reconstruction_time: q.DurationS = reconstructed["time"].to_numpy()
    cmp_mask = t <= reconstruction_time[-1]
    cmp_time: q.DurationS = t[cmp_mask]
    src_path = np.column_stack((lon[cmp_mask], lat[cmp_mask]))
    reconstructed_path = np.column_stack(
        (reconstructed["longitude"].to_numpy(), reconstructed["latitude"].to_numpy())
    )
    reconstructed_alt: q.PressureAltitudeM = np.interp(
        cmp_time, reconstruction_time, reconstructed["alt"].to_numpy()
    )
    reconstructed_cas: q.CalibratedAirspeedMps = np.interp(
        cmp_time, reconstruction_time, reconstructed["cas"].to_numpy()
    )
    reconstructed_mach: q.MachNumber = np.interp(
        cmp_time, reconstruction_time, reconstructed["mach"].to_numpy()
    )
    metrics = Metrics(
        sspd=float(traj_dist_rs.sspd(src_path, reconstructed_path, "spherical")),
        dtw=float(traj_dist_rs.dtw(src_path, reconstructed_path, "spherical").distance),
        alt_mae=float(
            q.m_to_ft(np.mean(np.abs(reconstructed_alt - source["alt"].to_numpy()[cmp_mask])))
        ),
        cas_mae=float(
            q.mps_to_kt(np.mean(np.abs(reconstructed_cas - source["cas"].to_numpy()[cmp_mask])))
        ),
        mach_mae=float(np.mean(np.abs(reconstructed_mach - source["mach"].to_numpy()[cmp_mask]))),
    )
    add_reconstruction_(axes, reconstructed, plan, metrics, source_start)
    axes.figure.savefig(str(output / "reconstruction.png"), dpi=300, bbox_inches="tight")
    plt.close(axes.figure)


if __name__ == "__main__":
    main()
