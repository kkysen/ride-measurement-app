// The elevators and escalators found (`ride_fusion::vertical::Kind::Riding`) against the rider's labels
// (`vertical_labels.csv`): a found move is a labeled one if it starts within 30 s of it.
use ride_fusion::replay::Replay;
use ride_fusion::vertical::Kind;

const LABELS: &str = include_str!("vertical_labels.csv");

pub fn run(name: &str, r: &Replay, out: &mut String) -> anyhow::Result<()> {
    let labels: Vec<(i64, &str, &str)> = LABELS
        .lines()
        .filter(|l| !l.starts_with('#') && !l.starts_with("session,"))
        .filter_map(|l| {
            let f: Vec<&str> = l.split(',').collect();
            (f[0] == name).then(|| Ok((f[1].parse()?, f[2], f[4])))
        })
        .collect::<anyhow::Result<_>>()?;
    let near = |a: i64, b: i64| (a - b).abs() <= 30_000_000_000;
    let label_of = |start: i64| labels.iter().find(|l| near(l.0, start)).map(|l| l.2);
    let found_as = |at: i64| {
        r.vertical
            .iter()
            .find(|m| near(m.start_ns, at))
            .map(|m| m.kind)
    };
    let riding: Vec<_> = r
        .vertical
        .iter()
        .filter(|m| m.kind == Kind::Riding)
        .collect();
    for m in &riding {
        out!(
            out,
            "riding {:+5.1} m at {}: {}",
            m.change_m,
            m.start_ns,
            label_of(m.start_ns).unwrap_or("unlabeled")
        );
    }
    let count = |label: &str| {
        riding
            .iter()
            .filter(|m| label_of(m.start_ns) == Some(label))
            .count()
    };
    let of = |label: &'static str| labels.iter().filter(move |l| l.2 == label);
    let elevators = of("elevator").count();
    let elevators_found = of("elevator")
        .filter(|l| found_as(l.0) == Some(Kind::Riding))
        .count();
    let stairs = of("stairs").count();
    let stairs_found = of("stairs")
        .filter(|l| found_as(l.0) == Some(Kind::Stairs))
        .count();
    out!(
        out,
        "vertical: elevators found {elevators_found}/{elevators}; riding found {} ({} elevator, {} stairs, {} none, {} unlabeled); exit stairs as stairs {stairs_found}/{stairs}",
        riding.len(),
        count("elevator"),
        count("stairs"),
        count("none"),
        riding
            .iter()
            .filter(|m| label_of(m.start_ns).is_none())
            .count()
    );
    Ok(())
}
