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