# BetaWolves Scouting System Design

## Overview

An offline-first scouting system for FRC competitions with three tiers:

1. **Scout PWA** - Mobile web app for data collection in the stands
2. **Hub Desktop App** (Tauri) - Laptop at team booth for aggregating data via QR codes
3. **Cloud** (Supabase + Next.js at betawolvesrobotics.com) - Central sync point when internet available

The system uses **UUIDs for deduplication**, **version numbers for conflict resolution**, and **QR codes for offline data transfer**.

## Constraints

- **No local WiFi** - FRC events prohibit teams from running their own wireless networks
- **Intermittent internet** - Venue WiFi is unreliable; cellular may be spotty
- **Mixed devices** - Scouts use personal phones (iOS/Android), need cross-platform solution
- **Speed matters** - Scouts have limited time between matches to submit data

---

## System Architecture

```
╔═══════════════════════════════════════════════════════════════════════════════╗
║                              CLOUD LAYER                                       ║
║                          (requires internet)                                   ║
╠═══════════════════════════════════════════════════════════════════════════════╣
║                                                                                ║
║   ┌─────────────────────────────┐         ┌─────────────────────────────┐    ║
║   │     betawolvesrobotics.com  │         │          Supabase           │    ║
║   │         (Next.js)           │◄───────►│                             │    ║
║   │                             │         │   ┌─────────────────────┐   │    ║
║   │   ┌───────────────────┐    │         │   │     Postgres        │   │    ║
║   │   │   Landing Page    │    │         │   │   ─────────────     │   │    ║
║   │   │   Team Info       │    │         │   │   scouting_entries  │   │    ║
║   │   └───────────────────┘    │         │   │   team_members      │   │    ║
║   │                             │         │   │   events            │   │    ║
║   │   ┌───────────────────┐    │         │   └─────────────────────┘   │    ║
║   │   │   /dashboard      │    │         │                             │    ║
║   │   │   (authenticated) │    │         │   ┌─────────────────────┐   │    ║
║   │   │   - Analytics     │    │         │   │     Supabase Auth   │   │    ║
║   │   │   - Pick Lists    │    │         │   │   (email/OAuth)     │   │    ║
║   │   │   - Admin         │    │         │   └─────────────────────┘   │    ║
║   │   └───────────────────┘    │         │                             │    ║
║   │                             │         │   ┌─────────────────────┐   │    ║
║   │   ┌───────────────────┐    │         │   │   Row Level Security │   │    ║
║   │   │   /api/sync       │    │         │   │   (team isolation)   │   │    ║
║   │   └───────────────────┘    │         │   └─────────────────────┘   │    ║
║   └─────────────────────────────┘                                            ║
║                                                                                ║
╚════════════════════════════════════╤══════════════════════════════════════════╝
                                     │
        ╔════════════════════════════╧════════════════════════════╗
        ║           INTERNET BOUNDARY (unreliable at events)      ║
        ╚════════════════════════════╤════════════════════════════╝
                                     │
╔════════════════════════════════════╧══════════════════════════════════════════╗
║                              EVENT LAYER                                       ║
║                      (offline, at competition venue)                           ║
╠═══════════════════════════════════════════════════════════════════════════════╣
║                                                                                ║
║   ┌───────────────────────────────────────────────────────────────────────┐  ║
║   │                      HUB DESKTOP APP (Tauri)                           │  ║
║   │                      Runs on team laptop at booth                      │  ║
║   │                                                                        │  ║
║   │   ┌─────────────┐   ┌─────────────┐   ┌─────────────┐                 │  ║
║   │   │   Webcam    │   │   SQLite    │   │   Cloud     │                 │  ║
║   │   │   Scanner   │   │   Database  │   │   Sync      │◄──── When online │  ║
║   │   └──────┬──────┘   └──────┬──────┘   └─────────────┘                 │  ║
║   │          │                 │                                           │  ║
║   │          │    ┌────────────┴────────────┐                             │  ║
║   │          └───►│    Entry Manager        │                             │  ║
║   │               │    - Deduplicate        │                             │  ║
║   │               │    - Resolve conflicts  │                             │  ║
║   │               └────────────┬────────────┘                             │  ║
║   │                            │                                           │  ║
║   │               ┌────────────┴────────────┐                             │  ║
║   │               │    QR Code Display      │◄──── Shows QR for scouts    │  ║
║   │               │    (response to scouts) │      to scan back           │  ║
║   │               └─────────────────────────┘                             │  ║
║   └───────────────────────────────────────────────────────────────────────┘  ║
║                                        ▲                                      ║
║                         ┌──────────────┼──────────────┐                      ║
║                         │     QR Code  │  Transfer    │                      ║
║                         │   (bidirectional sync)      │                      ║
║                         ▼              ▼              ▼                      ║
║   ┌──────────────────────────────────────────────────────────────────────┐  ║
║   │   ┌──────────────┐   ┌──────────────┐   ┌──────────────┐             │  ║
║   │   │  Scout PWA   │   │  Scout PWA   │   │  Scout PWA   │    ...      │  ║
║   │   │  (Phone 1)   │   │  (Phone 2)   │   │  (Phone 3)   │             │  ║
║   │   │ ┌──────────┐ │   │ ┌──────────┐ │   │ ┌──────────┐ │             │  ║
║   │   │ │IndexedDB │ │   │ │IndexedDB │ │   │ │IndexedDB │ │             │  ║
║   │   │ │QR Gen/Scan│ │   │ │QR Gen/Scan│ │   │ │QR Gen/Scan│ │             │  ║
║   │   │ │Forms     │ │   │ │Forms     │ │   │ │Forms     │ │             │  ║
║   │   │ └──────────┘ │   │ └──────────┘ │   │ └──────────┘ │             │  ║
║   │   └──────────────┘   └──────────────┘   └──────────────┘             │  ║
║   └──────────────────────────────────────────────────────────────────────┘  ║
╚═══════════════════════════════════════════════════════════════════════════════╝
```

---

## Data Flow Scenarios

### Scenario 1: Scout Collects Data (Offline)

1. Scout opens PWA on their phone
2. Selects: Event → Match → Team to scout
3. Fills scouting form during match (tap counters, toggles)
4. Form auto-saves every 5 seconds to IndexedDB
5. Taps "Save" after match

```
Entry stored locally:
{
  id: "550e8400-e29b-41d4-a716-446655440000",  // UUID
  device_id: "device-abc123",
  scout_name: "Alex",
  match_key: "2026txhou_qm15",
  team_number: 6637,
  data: { ... },
  version: 1,
  _sync_status: "pending"  ← Not yet synced
}
```

### Scenario 2: Scout → Hub Sync (QR Code)

**Step 1: Scout generates QR**

- Scout taps "Sync" button
- App compresses pending entries → generates QR code
- Scout walks to hub station

**Step 2: Hub scans**

- Hub webcam scans scout's QR code
- Hub decodes, upserts entries to SQLite
- Hub beeps: "Got 1 entry from Alex"

**Step 3: Hub responds (bidirectional)**

- Hub queries entries this scout doesn't have
- Hub displays QR code on laptop screen
- Scout uses phone camera to scan hub's QR
- Scout now has data from other scouts

### Scenario 3: Hub → Cloud Sync

When hub gets internet (phone hotspot, venue WiFi):

1. **PUSH**: Hub sends all unsynced entries to Supabase (upsert by UUID)
2. **PULL**: Hub fetches entries from cloud it doesn't have
3. **MARK**: Hub marks everything as cloud-synced

### Scenario 4: Multiple Hubs / Eventual Consistency

Two hubs at event collecting from different scouts:

- Hub A collects matches 1-10 (25 entries)
- Hub B collects matches 11-20 (30 entries)
- Hub A goes online first → pushes 25 to cloud
- Hub B goes online later → pushes 30, pulls 25
- Both now have 55 entries, no duplicates (UUIDs)

---

## Data Model

### ScoutingEntry

```typescript
interface ScoutingEntry {
  // ═══════════════════════════════════════════════════════════════
  // IDENTITY (immutable after creation)
  // ═══════════════════════════════════════════════════════════════
  id: string; // UUID v4, client-generated (deduplication key)
  device_id: string; // Unique per device installation
  scout_name: string; // Who collected this data

  // ═══════════════════════════════════════════════════════════════
  // MATCH IDENTIFICATION
  // ═══════════════════════════════════════════════════════════════
  event_key: string; // TBA event key, e.g., "2026txhou"
  match_key: string; // TBA match key, e.g., "2026txhou_qm15"
  match_type: "practice" | "qualification" | "elimination";
  match_number: number;
  team_number: number; // Team being scouted
  alliance: "red" | "blue";
  alliance_position: 1 | 2 | 3;

  // ═══════════════════════════════════════════════════════════════
  // SCOUTING DATA (game-specific, changes yearly)
  // ═══════════════════════════════════════════════════════════════
  auto_data: AutoPhaseData;
  teleop_data: TeleopPhaseData;
  endgame_data: EndgameData;
  ratings: RobotRatings;
  notes: string;

  // ═══════════════════════════════════════════════════════════════
  // SYNC METADATA
  // ═══════════════════════════════════════════════════════════════
  created_at: string; // ISO 8601, set once
  updated_at: string; // Updated on every edit (conflict resolution)
  version: number; // Incremented on edits, higher wins

  // ═══════════════════════════════════════════════════════════════
  // LOCAL-ONLY FIELDS (not synced to cloud)
  // ═══════════════════════════════════════════════════════════════
  _sync_status: "pending" | "synced_to_hub" | "synced_to_cloud";
  _synced_at: string | null;
}
```

### Game-Specific Data (2026 REEFSCAPE)

```typescript
interface AutoPhaseData {
  starting_position: "left" | "center" | "right";
  left_starting_zone: boolean;
  coral_scored: { l1: number; l2: number; l3: number; l4: number };
  algae_removed_from_reef: number;
  algae_scored_processor: number;
  algae_scored_net: number;
}

interface TeleopPhaseData {
  coral_scored: { l1: number; l2: number; l3: number; l4: number };
  coral_dropped: number;
  algae_removed_from_reef: number;
  algae_scored_processor: number;
  algae_scored_net: number;
  algae_dropped: number;
  human_player_accuracy: "poor" | "average" | "good" | "excellent";
}

interface EndgameData {
  cage_climb_attempted: boolean;
  cage_climb_level: "none" | "park" | "shallow" | "deep";
  cage_climb_success: boolean;
  cage_climb_time_seconds: number | null;
  barge_parked: boolean;
}

interface RobotRatings {
  defense_rating: 1 | 2 | 3 | 4 | 5;
  driver_skill: 1 | 2 | 3 | 4 | 5;
  robot_speed: 1 | 2 | 3 | 4 | 5;
  robot_stability: 1 | 2 | 3 | 4 | 5;
  overall_impression: 1 | 2 | 3 | 4 | 5;
}
```

---

## Database Schemas

### Supabase (Postgres)

```sql
-- Team & User Management
CREATE TABLE teams (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  team_number INTEGER UNIQUE NOT NULL,
  team_name TEXT NOT NULL,
  created_at TIMESTAMPTZ DEFAULT NOW()
);

CREATE TABLE team_members (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  user_id UUID REFERENCES auth.users(id) ON DELETE CASCADE,
  team_id UUID REFERENCES teams(id) ON DELETE CASCADE,
  role TEXT NOT NULL DEFAULT 'scout',  -- 'scout', 'lead', 'admin'
  display_name TEXT NOT NULL,
  created_at TIMESTAMPTZ DEFAULT NOW(),
  UNIQUE(user_id, team_id)
);

-- Events
CREATE TABLE events (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  team_id UUID REFERENCES teams(id) ON DELETE CASCADE,
  event_key TEXT NOT NULL,
  event_name TEXT NOT NULL,
  start_date DATE NOT NULL,
  end_date DATE NOT NULL,
  created_at TIMESTAMPTZ DEFAULT NOW(),
  UNIQUE(team_id, event_key)
);

-- Scouting Entries
CREATE TABLE scouting_entries (
  id UUID PRIMARY KEY,               -- Client-generated UUID
  device_id TEXT NOT NULL,
  scout_name TEXT NOT NULL,
  team_id UUID REFERENCES teams(id) ON DELETE CASCADE,

  event_key TEXT NOT NULL,
  match_key TEXT NOT NULL,
  match_type TEXT NOT NULL,
  match_number INTEGER NOT NULL,
  team_number INTEGER NOT NULL,
  alliance TEXT NOT NULL,
  alliance_position INTEGER NOT NULL,

  auto_data JSONB NOT NULL,
  teleop_data JSONB NOT NULL,
  endgame_data JSONB NOT NULL,
  ratings JSONB NOT NULL,
  notes TEXT,

  created_at TIMESTAMPTZ NOT NULL,
  updated_at TIMESTAMPTZ NOT NULL,
  version INTEGER NOT NULL DEFAULT 1,

  CONSTRAINT valid_alliance CHECK (alliance IN ('red', 'blue')),
  CONSTRAINT valid_position CHECK (alliance_position BETWEEN 1 AND 3)
);

-- Indexes
CREATE INDEX idx_entries_team_event ON scouting_entries(team_id, event_key);
CREATE INDEX idx_entries_event_match ON scouting_entries(event_key, match_number);
CREATE INDEX idx_entries_team_number ON scouting_entries(event_key, team_number);
CREATE INDEX idx_entries_updated ON scouting_entries(team_id, updated_at);

-- Row Level Security
ALTER TABLE scouting_entries ENABLE ROW LEVEL SECURITY;

CREATE POLICY "Users can view own team entries" ON scouting_entries FOR SELECT
  USING (team_id IN (SELECT team_id FROM team_members WHERE user_id = auth.uid()));

CREATE POLICY "Users can insert own team entries" ON scouting_entries FOR INSERT
  WITH CHECK (team_id IN (SELECT team_id FROM team_members WHERE user_id = auth.uid()));

CREATE POLICY "Users can update own team entries" ON scouting_entries FOR UPDATE
  USING (team_id IN (SELECT team_id FROM team_members WHERE user_id = auth.uid()));

-- Upsert Function (version check for conflict resolution)
CREATE OR REPLACE FUNCTION upsert_scouting_entries(entries JSONB)
RETURNS TABLE(id UUID, status TEXT) AS $$
DECLARE
  entry JSONB;
BEGIN
  FOR entry IN SELECT * FROM jsonb_array_elements(entries) LOOP
    INSERT INTO scouting_entries (
      id, device_id, scout_name, team_id,
      event_key, match_key, match_type, match_number,
      team_number, alliance, alliance_position,
      auto_data, teleop_data, endgame_data, ratings, notes,
      created_at, updated_at, version
    ) VALUES (
      (entry->>'id')::UUID, entry->>'device_id', entry->>'scout_name',
      (entry->>'team_id')::UUID, entry->>'event_key', entry->>'match_key',
      entry->>'match_type', (entry->>'match_number')::INTEGER,
      (entry->>'team_number')::INTEGER, entry->>'alliance',
      (entry->>'alliance_position')::INTEGER,
      entry->'auto_data', entry->'teleop_data', entry->'endgame_data',
      entry->'ratings', entry->>'notes',
      (entry->>'created_at')::TIMESTAMPTZ, (entry->>'updated_at')::TIMESTAMPTZ,
      (entry->>'version')::INTEGER
    )
    ON CONFLICT (id) DO UPDATE SET
      auto_data = EXCLUDED.auto_data,
      teleop_data = EXCLUDED.teleop_data,
      endgame_data = EXCLUDED.endgame_data,
      ratings = EXCLUDED.ratings,
      notes = EXCLUDED.notes,
      updated_at = EXCLUDED.updated_at,
      version = EXCLUDED.version
    WHERE EXCLUDED.version > scouting_entries.version;

    id := (entry->>'id')::UUID;
    status := CASE WHEN FOUND THEN 'upserted' ELSE 'skipped' END;
    RETURN NEXT;
  END LOOP;
END;
$$ LANGUAGE plpgsql SECURITY DEFINER;
```

### Hub SQLite Schema

```sql
CREATE TABLE scouting_entries (
  id TEXT PRIMARY KEY,
  device_id TEXT NOT NULL,
  scout_name TEXT NOT NULL,
  team_id TEXT NOT NULL,

  event_key TEXT NOT NULL,
  match_key TEXT NOT NULL,
  match_type TEXT NOT NULL,
  match_number INTEGER NOT NULL,
  team_number INTEGER NOT NULL,
  alliance TEXT NOT NULL,
  alliance_position INTEGER NOT NULL,

  auto_data TEXT NOT NULL,       -- JSON string
  teleop_data TEXT NOT NULL,
  endgame_data TEXT NOT NULL,
  ratings TEXT NOT NULL,
  notes TEXT,

  created_at TEXT NOT NULL,
  updated_at TEXT NOT NULL,
  version INTEGER NOT NULL DEFAULT 1,

  received_from_device TEXT,
  received_at TEXT,
  synced_to_cloud_at TEXT        -- NULL if not yet synced
);

CREATE INDEX idx_entries_event ON scouting_entries(event_key);
CREATE INDEX idx_entries_unsynced ON scouting_entries(synced_to_cloud_at)
  WHERE synced_to_cloud_at IS NULL;
```

---

## QR Code Protocol

### Payload Structure

```typescript
interface QRPayload {
  v: 1; // Protocol version
  t: QRPayloadType; // Message type
  p: number; // Part number (1-indexed)
  n: number; // Total parts
  d: string; // Compressed data (LZ-String Base64)
  c: string; // Checksum (first 8 chars of SHA-256)
}

type QRPayloadType = "scout_push" | "scout_pull_request" | "hub_response";
```

### Message Types

```typescript
// Scout → Hub: Push entries
interface ScoutPushPayload {
  type: "scout_push";
  device_id: string;
  scout_name: string;
  team_id: string;
  last_hub_sync_at: string | null;
  entries: ScoutingEntry[];
}

// Hub → Scout: Response with entries scout is missing
interface HubResponsePayload {
  type: "hub_response";
  for_device_id: string;
  ack_count: number; // How many entries hub received
  entries: ScoutingEntry[]; // Entries scout doesn't have
}
```

### Encoding Implementation

```typescript
import lzstring from "lz-string";
import { sha256 } from "js-sha256";

const MAX_QR_CHARS = 1800; // Leave room for wrapper

export function encodePayload<T>(payload: T): string[] {
  const json = JSON.stringify(payload);
  const checksum = sha256(json).substring(0, 8);
  const compressed = lzstring.compressToBase64(json);

  const chunks: string[] = [];
  const totalParts = Math.ceil(compressed.length / MAX_QR_CHARS);

  for (let i = 0; i < compressed.length; i += MAX_QR_CHARS) {
    const qrPayload: QRPayload = {
      v: 1,
      t: (payload as any).type,
      p: Math.floor(i / MAX_QR_CHARS) + 1,
      n: totalParts,
      d: compressed.slice(i, i + MAX_QR_CHARS),
      c: checksum,
    };
    chunks.push(JSON.stringify(qrPayload));
  }
  return chunks;
}

export function decodePayload<T>(qrPayloads: QRPayload[]): T {
  if (qrPayloads.length !== qrPayloads[0].n) {
    throw new Error(
      `Missing parts: got ${qrPayloads.length}, expected ${qrPayloads[0].n}`,
    );
  }

  qrPayloads.sort((a, b) => a.p - b.p);
  const compressed = qrPayloads.map((p) => p.d).join("");
  const json = lzstring.decompressFromBase64(compressed);

  if (!json) throw new Error("Decompression failed");

  const actualChecksum = sha256(json).substring(0, 8);
  if (actualChecksum !== qrPayloads[0].c) {
    throw new Error("Checksum mismatch");
  }

  return JSON.parse(json) as T;
}
```

### Typical Payload Sizes

| Entries | Raw JSON | Compressed | QR Codes |
| ------- | -------- | ---------- | -------- |
| 1       | ~500 B   | ~300 B     | 1        |
| 5       | ~2.5 KB  | ~1.2 KB    | 1        |
| 10      | ~5 KB    | ~2.2 KB    | 2        |
| 20      | ~10 KB   | ~4 KB      | 3        |

Scouts typically sync after each match (1 entry) = single QR code.

---

## User Experience

### Scout Flow

1. **Open app** → Select event (cached) → Select match number
2. **Pre-match** → Pick team to scout, confirm alliance color
3. **During match** → Tap counters/toggles, form auto-saves every 5s
4. **Post-match** → Add notes, tap "Save"
5. **Sync** → Walk to hub, tap "Sync", show QR, wait for beep, scan hub's response QR

### Hub Operator Flow

1. **Start hub app** → Select event
2. **Continuous scanning** → Webcam always watching for QR codes
3. **On scan** → Beep, show "Got 1 entry from [Scout Name]"
4. **Display response** → Show QR for scout to scan back
5. **When online** → Sync to cloud, view analytics

---

## Tech Stack

| Component     | Technology                 | Rationale                                                |
| ------------- | -------------------------- | -------------------------------------------------------- |
| Scout App     | **PWA** (SvelteKit)        | Cross-platform, no app store, installable, camera access |
| Local Storage | **IndexedDB** via Dexie.js | Large capacity, structured queries, offline              |
| QR Generation | `qrcode` npm               | Reliable, customizable                                   |
| QR Scanning   | `html5-qrcode`             | Works in mobile browsers                                 |
| Compression   | `lz-string`                | Good JSON compression, Base64 output                     |
| Hub App       | **Tauri**                  | Native perf, reliable webcam, small binary (~10MB)       |
| Hub Database  | **SQLite**                 | Fast, single-file, easy backup                           |
| Cloud         | **Supabase**               | Postgres, Auth, RLS, good free tier                      |
| Web App       | **Next.js**                | Dashboard, analytics, API routes                         |
| Styling       | **Tailwind CSS**           | Fast development                                         |

---

## Offline-First Principles

1. **Local-first** - All writes go to local storage first, always
2. **Sync is secondary** - App works indefinitely without syncing
3. **Idempotent sync** - Can scan same QR multiple times safely (UUID dedup)
4. **Conflict resolution** - Higher `version` number wins
5. **Eventual consistency** - All devices converge to same state

---

## Security

- **No auth at event** - Scouts pre-register, use name selection (no passwords in stands)
- **Auth for cloud** - Supabase auth when syncing to cloud
- **Team isolation** - RLS ensures users only see their team's data
- **Event scoping** - Entries scoped by event_key
- **No PII** - Only scout names (team members)

---

## Project Structure

```
betawolves-scouting/
├── apps/
│   ├── web/                          # Next.js (betawolvesrobotics.com)
│   │   ├── app/
│   │   │   ├── (marketing)/          # Landing, public pages
│   │   │   ├── (auth)/               # Login, signup
│   │   │   ├── (dashboard)/          # Authenticated area
│   │   │   │   ├── scouting/         # View scouting data
│   │   │   │   ├── analytics/        # Charts, pick lists
│   │   │   │   └── admin/            # Team management
│   │   │   └── api/sync/             # Sync endpoints
│   │   └── package.json
│   │
│   ├── scout-pwa/                    # SvelteKit PWA
│   │   ├── src/
│   │   │   ├── lib/
│   │   │   │   ├── db/               # Dexie IndexedDB
│   │   │   │   ├── qr/               # QR encode/decode/scan
│   │   │   │   └── sync/             # Sync manager
│   │   │   ├── routes/
│   │   │   │   ├── scout/[matchKey]/ # Scouting form
│   │   │   │   ├── sync/             # QR display/scan
│   │   │   │   └── history/          # Past entries
│   │   │   └── service-worker.ts
│   │   └── package.json
│   │
│   └── hub-desktop/                  # Tauri app
│       ├── src/                      # Web frontend
│       │   ├── lib/scanner/          # Webcam QR
│       │   └── routes/
│       ├── src-tauri/                # Rust backend
│       │   ├── src/
│       │   │   ├── db.rs             # SQLite
│       │   │   └── sync.rs           # Cloud sync
│       │   └── Cargo.toml
│       └── package.json
│
├── packages/
│   ├── shared/                       # Shared types, QR protocol
│   │   ├── src/
│   │   │   ├── types/
│   │   │   ├── qr/
│   │   │   └── validation/
│   │   └── package.json
│   └── ui/                           # Shared components
│
├── supabase/
│   ├── migrations/
│   └── config.toml
│
├── package.json
├── pnpm-workspace.yaml
└── turbo.json
```

---

## Decision Log

| Decision    | Choice           | Rationale                                    |
| ----------- | ---------------- | -------------------------------------------- |
| Scout app   | PWA              | Cross-platform, no app store, camera works   |
| Hub app     | Tauri            | Native webcam, SQLite, small binary          |
| Sync method | QR codes         | Works offline, no WiFi needed, proven in FRC |
| Dedup key   | UUID             | Client-generated, globally unique            |
| Conflicts   | Version number   | Simple, deterministic                        |
| Cloud DB    | Supabase         | Free tier, RLS, good DX                      |
| Monorepo    | Turborepo + pnpm | Fast builds, workspace support               |

---

## Next Steps

1. [ ] Set up monorepo with Turborepo + pnpm
2. [ ] Create `packages/shared` with types and QR protocol
3. [ ] Set up Supabase project with schema
4. [ ] Build scout-pwa with basic form and QR generation
5. [ ] Build hub-desktop with webcam scanning
6. [ ] Implement bidirectional sync
7. [ ] Build web dashboard for analytics
8. [ ] Test at practice sessions
