export const Button = ({ children = "Button", filled = true, accentColor = "#3487b7ff", accentTextColor = "#ffffff", onClick = () => { } }) => {
    return (
        <div className="v-btn" style={{
            background: filled ? accentColor : "transparent",
            color: filled ? accentTextColor : "black"
        }} onClick={onClick}>
            {children}
        </div>
    );
}

export const DictDisplay = ({ children = {}, hint = "" }) => {
    return (<div className="v-dict-display">
        {
            Object.keys(children).map((key) => (<div className="v-dict-display--item">
                <p className="v-dict-display--key">{key}</p>
                :
                <p className="v-dict-display--value">{children[key]}</p>
            </div>))
        }

        <div className="v-dict-display--hint">{hint}</div>
    </div>);
}

export const MapDisplay = ({ map, path, mapSizePx = 10000 }) => {
    return (<div className="v-map-display">
        
    </div>);
}